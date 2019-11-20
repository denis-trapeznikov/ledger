//------------------------------------------------------------------------------
//
//   Copyright 2018-2019 Fetch.AI Limited
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//
//------------------------------------------------------------------------------

#include "bloom_filter/bloom_filter.hpp"
#include "chain/transaction_layout_rpc_serializers.hpp"
#include "core/assert.hpp"
#include "core/byte_array/byte_array.hpp"
#include "core/byte_array/encoders.hpp"
#include "crypto/hash.hpp"
#include "crypto/sha256.hpp"
#include "ledger/chain/block_db_record.hpp"
#include "ledger/chain/main_chain.hpp"
#include "ledger/chain/time_travelogue.hpp"
#include "network/generics/milli_timer.hpp"
#include "telemetry/counter.hpp"
#include "telemetry/gauge.hpp"
#include "telemetry/registry.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using fetch::byte_array::ToBase64;
using fetch::generics::MilliTimer;

namespace fetch {
namespace ledger {

/**
 * Constructs the main chain
 *
 * @param mode Flag to signal which storage mode has been requested
 */
MainChain::MainChain(bool const enable_bloom_filter, Mode mode)
  : bloom_filter_{std::make_unique<BasicBloomFilter>()}
  , enable_bloom_filter_{enable_bloom_filter}
  , bloom_filter_queried_bit_count_(telemetry::Registry::Instance().CreateGauge<std::size_t>(
        "ledger_main_chain_bloom_filter_queried_bit_number",
        "Total number of bits checked during each query to the Ledger Main Chain Bloom filter"))
  , bloom_filter_query_count_(telemetry::Registry::Instance().CreateCounter(
        "ledger_main_chain_bloom_filter_query_total",
        "Total number of queries to the Ledger Main Chain Bloom filter"))
  , bloom_filter_positive_count_(telemetry::Registry::Instance().CreateCounter(
        "ledger_main_chain_bloom_filter_positive_total",
        "Total number of positive queries (false and true) to the Ledger Main Chain Bloom filter"))
  , bloom_filter_false_positive_count_(telemetry::Registry::Instance().CreateCounter(
        "ledger_main_chain_bloom_filter_false_positive_total",
        "Total number of false positive queries to the Ledger Main Chain Bloom filter"))
{
  if (Mode::IN_MEMORY_DB != mode)
  {
    // create the block store
    block_store_ = std::make_unique<BlockStore>();

    RecoverFromFile(mode);
  }

  // create the genesis block and add it to the cache
  auto genesis = CreateGenesisBlock();

  // add the block to the cache
  AddBlockToCache(genesis);

  // add the tip for this block
  AddTip(genesis);
}

MainChain::~MainChain()
{
  if (block_store_)
  {
    block_store_->Flush(false);
  }
}

void MainChain::Reset()
{
  FETCH_LOCK(lock_);

  tips_.clear();
  heaviest_ = HeaviestTip{};
  loose_blocks_.clear();
  block_chain_.clear();
  forward_references_.clear();

  if (block_store_)
  {
    block_store_->New("chain.db", "chain.index.db");
    head_store_.close();
    head_store_.open("chain.head.db",
                     std::ios::binary | std::ios::in | std::ios::out | std::ios::trunc);
  }

  auto genesis = CreateGenesisBlock();

  // add the block to the cache
  AddBlockToCache(genesis);

  // add the tip for this block
  AddTip(genesis);
}

/**
 * Adds a block to the chain
 *
 * @param block The block that will be added to the chain.
 * @return The status enumeration signalling the operation result
 */
BlockStatus MainChain::AddBlock(Block const &blk)
{
  // create a copy of the block
  auto block = std::make_shared<Block>(blk);

  // At this point we assume that the weight has been correctly set by the miner
  block->total_weight = 1;

  auto const status = InsertBlock(block);
  FETCH_LOG_DEBUG(LOGGING_NAME, "New Block: 0x", block->hash.ToHex(), " -> ", ToString(status),
                  " (weight: ", block->weight, " total: ", block->total_weight, ")");

  if (status == BlockStatus::ADDED)
  {
    AddBlockToBloomFilter(*block);
  }

  return status;
}

/**
 * Internal: add a parent-child forward reference if it is unknown yet.
 * Update parent block, if found, with the relevant forward information.
 *
 * @param hash
 * @param next_hash
 * @param unique if set to true, forget all other references, if any
 */
void MainChain::CacheReference(BlockHash const &hash, BlockHash const &next_hash, bool unique) const
{
  // get all known forward references range for this parent
  auto siblings = forward_references_.equal_range(hash);
  // references from storage are unique
  if (unique)
  {
    // forget all other references
    forward_references_.erase(siblings.first, siblings.second);
    // keep unique this one
    forward_references_.emplace(hash, next_hash);
    return;
  }

  // check if this parent-child reference has been already cached
  auto ref_it = std::find_if(siblings.first, siblings.second,
                             [&next_hash](auto const &ref) { return ref.second == next_hash; });
  if (ref_it == siblings.second)
  {
    // this child has not been already referred to yet
    forward_references_.emplace_hint(siblings.first, hash, next_hash);
  }
}

/**
 * Internal: forget a parent-child forward reference.
 * Update parent block, if found, with the relevant forward information.
 *
 * @param hash
 * @param next_hash
 */
void MainChain::ForgetReference(BlockHash const &hash, BlockHash const &next_hash) const
{
  // get all known forward references range for this parent
  auto siblings = forward_references_.equal_range(hash);
  if (next_hash.empty())
  {
    // no particular child specified, forget all references from this parent
    forward_references_.erase(siblings.first, siblings.second);
    return;
  }
  // find a particular reference to this child
  auto ref_it = std::find_if(siblings.first, siblings.second,
                             [&next_hash](auto const &ref) { return ref.second == next_hash; });
  if (ref_it != siblings.second)
  {
    forward_references_.erase(ref_it);
  }
}

/**
 * Lookup next_hash for a block in-memory.
 * This procedure may fail in which case next_hash should either be looked for in storage,
 * or recovered by descending from tip.
 *
 * @param hash Hash of the current block
 * @param[out] next_hash Hash of unique block grown from this
 * @return true iff a single forward reference is found in cache
 */
bool MainChain::LookupReference(BlockHash const &hash, BlockHash &next_hash) const
{
  switch (forward_references_.count(hash))
  {
  case 0:
    // should be a tip
    next_hash = BlockHash{};
    return true;
  case 1:
    // unique reference
    next_hash = forward_references_.find(hash)->second;
    return true;
  default:
    // ambiguous forward references need to be resolved from tip
    return false;
  }
}

/**
 * Internal: insert a block into the cache maintaining references
 *
 * @param block The block to be cached
 */
void MainChain::CacheBlock(IntBlockPtr const &block) const
{
  ASSERT(static_cast<bool>(block));

  auto const &hash = block->hash;
  auto        ret_val{block_chain_.emplace(hash, block)};

  // under all circumstances, it _should_ be a fresh block
  ASSERT(ret_val.second);
  if (!block->IsGenesis())
  {
    // keep parent-child reference
    CacheReference(block->previous_hash, hash);
  }
}

/**
 * Internal: erase a block from the cache
 *
 * @param hash The hash of the block to be erased
 * @return amount of blocks erased (1 or 0, if not found)
 */
MainChain::BlockMap::size_type MainChain::UncacheBlock(BlockHash const &hash) const
{
  return block_chain_.erase(hash);
  // references are kept intact while this cache is alive
}

/**
 * Internal: insert a block into the permanent store maintaining references
 *
 * @param block The block to be kept
 */
void MainChain::KeepBlock(IntBlockPtr const &block) const
{
  ASSERT(static_cast<bool>(block));
  ASSERT(static_cast<bool>(block_store_));

  auto const &hash{block->hash};

  DbRecord record;

  if (!block->IsGenesis())
  {
    // notify stored parent
    if (block_store_->Get(storage::ResourceID(block->previous_hash), record))
    {
      if (record.next_hash != hash)
      {
        record.next_hash = hash;
        block_store_->Set(storage::ResourceID(record.hash()), record);
      }
      // before checking for this block's children in storage, reset next_hash to genesis
      record.next_hash = Digest{};
      CacheReference(block->previous_hash, hash, true);
    }
  }
  record.block = *block;

  // detect if any of this block's children has made it to the store already
  auto forward_refs{forward_references_.equal_range(hash)};
  for (auto ref_it{forward_refs.first}; ref_it != forward_refs.second; ++ref_it)
  {
    auto const &child{ref_it->second};
    if (block_store_->Has(storage::ResourceID(child)))
    {
      record.next_hash = child;
      CacheReference(hash, child, true);
      break;
    }
  }

  // now write the block itself; if next_hash is genesis, it will be rewritten later by a child
  block_store_->Set(storage::ResourceID(hash), record);
}

/**
 * Internal: load a block from the permanent store
 *
 * @param[in]  hash The hash of the block to be loaded
 * @param[out] block The location of block
 * @return True iff the block is found in the storage
 */
bool MainChain::LoadBlock(BlockHash const &hash, Block &block, BlockHash *next_hash) const
{
  assert(static_cast<bool>(block_store_));

  DbRecord record;
  if (block_store_->Get(storage::ResourceID(hash), record))
  {
    block = record.block;
    AddBlockToBloomFilter(block);
    if (next_hash != nullptr)
    {
      *next_hash = record.next_hash;
    }
    // update references assuming those from storage are unique
    if (!block.IsGenesis())
    {
      CacheReference(block.previous_hash, hash, true);
    }
    if (!record.next_hash.empty())
    {
      CacheReference(hash, record.next_hash, true);
    }

    return true;
  }

  return false;
}

void MainChain::AddBlockToBloomFilter(Block const &block) const
{
  for (auto const &slice : block.slices)
  {
    for (auto const &tx : slice)
    {
      bloom_filter_->Add(tx.digest());
    }
  }
}

/**
 * Get the current heaviest block on the chain
 *
 * @return The reference to the heaviest block
 */
MainChain::BlockPtr MainChain::GetHeaviestBlock() const
{
  FETCH_LOCK(lock_);
  auto block_ptr = GetBlock(heaviest_.hash);
  assert(block_ptr);
  return block_ptr;
}

/**
 * Internal: remove the block, and all blocks ahead of it, and references between them, from the
 * cache.
 *
 * @param[in]  hash The hash to be removed
 * @param[out] invalidated_blocks The set of hashes of all the blocks removed by this operation
 * @return The block object itself, already not in the cache
 */
bool MainChain::RemoveTree(BlockHash const &removed_hash, BlockHashSet &invalidated_blocks)
{
  // check if the block is actually found in this chain
  IntBlockPtr root;
  bool        retVal{LookupBlock(removed_hash, root)};
  if (retVal)
  {
    // forget the forward ref to this block from its parent
    auto siblings{forward_references_.equal_range(root->previous_hash)};
    for (auto sibling = siblings.first; sibling != siblings.second; ++sibling)
    {
      if (sibling->second == removed_hash)
      {
        forward_references_.erase(sibling);
        break;
      }
    }
  }

  for (BlockHashes next_gen{removed_hash}; !next_gen.empty();)
  {
    // when next_gen becomes current generation, next generation is clear
    for (auto const &hash : std::exchange(next_gen, BlockHashes{}))
    {
      // mark hash as being invalidated
      invalidated_blocks.insert(hash);

      // first remember to remove all the progeny breadth-first
      // this way any hash that could potentially stem from this one,
      // reference tree-wise, is completely wiped out
      auto children{forward_references_.equal_range(hash)};
      for (auto child{children.first}; child != children.second; ++child)
      {
        next_gen.push_back(child->second);
      }
      forward_references_.erase(children.first, children.second);

      // next, remove the block record from the cache, if found
      if (block_chain_.erase(hash) != 0u)
      {
        retVal = true;
      }
    }
  }

  return retVal;
}

/**
 * Removes the block (and all blocks ahead of it) from the chain.
 *
 * @param hash The hash to be removed
 * @return True if successful, otherwise false
 */
bool MainChain::RemoveBlock(BlockHash const &hash)
{
  FETCH_LOCK(lock_);

  // Step 0. Manually set heaviest to a block we still know is valid
  auto block_to_remove = GetBlock(hash);

  if (!block_to_remove)
  {
    return false;
  }

  if (block_to_remove->IsGenesis())
  {
    FETCH_LOG_ERROR(LOGGING_NAME, "Trying to remove genesis block!");
    return false;
  }

  // if block is not loose update heaviest_
  auto loose_it = loose_blocks_.find(hash);
  if (loose_it == loose_blocks_.end())
  {
    auto block_before_one_to_del = GetBlock(block_to_remove->previous_hash);
    if (block_before_one_to_del)
    {
      heaviest_ = HeaviestTip{};
      heaviest_.Update(*block_before_one_to_del);
    }
  }

  // Step 1. Remove this block and the whole its progeny from the cache
  BlockHashSet invalidated_blocks;
  if (!RemoveTree(hash, invalidated_blocks))
  {
    // no blocks were removed during this attempt
    return false;
  }

  // Step 2. Cleanup loose blocks
  for (auto waiting_blocks_it{loose_blocks_.begin()}; waiting_blocks_it != loose_blocks_.end();)
  {
    auto &hash_array{waiting_blocks_it->second};
    // remove entries from the hash array that have been invalidated
    auto cemetery{std::remove_if(hash_array.begin(), hash_array.end(),
                                 [&invalidated_blocks](auto const &hash) {
                                   return invalidated_blocks.find(hash) != invalidated_blocks.end();
                                 })};
    if (cemetery == hash_array.begin())
    {
      // all hashes in this array are invalidated
      waiting_blocks_it = loose_blocks_.erase(waiting_blocks_it);
    }
    else
    {
      // some hashes of this array are still alive
      hash_array.erase(cemetery, hash_array.end());
      ++waiting_blocks_it;
    }
  }

  // Step 3. Since we might have removed a whole series of blocks the tips data structure
  // is likely to have been invalidated. We need to evaluate the changes here
  return ReindexTips();
}

/**
 * Walk the block history starting from the heaviest block
 *
 * @param lowest_block_number The minimum allowed block_number to be returned
 * @return The array of blocks
 * @throws std::runtime_error if a block lookup occurs
 */
MainChain::Blocks MainChain::GetHeaviestChain(uint64_t lowest_block_number) const
{
  // Note: min needs a reference to something, so this is a workaround since UPPER_BOUND is a
  // constexpr
  MilliTimer myTimer("MainChain::HeaviestChain");

  FETCH_LOCK(lock_);

  return GetChainPreceding(GetHeaviestBlockHash(), lowest_block_number);
}

/**
 * Walk the block history collecting blocks until either genesis or the block limit is reached
 *
 * @param start The hash of the first block
 * @param lowest_block_number The minimum allowed block_number to be returned
 * @return The array of blocks
 * @throws std::runtime_error if a block lookup occurs
 */
MainChain::Blocks MainChain::GetChainPreceding(BlockHash start, uint64_t lowest_block_number) const
{
  MilliTimer myTimer("MainChain::ChainPreceding");

  FETCH_LOCK(lock_);

  // asserting genesis block has a number of 0, and everything else is above
  assert(GetBlock(chain::GENESIS_HASH));
  assert(GetBlock(chain::GENESIS_HASH)->block_number == 0);
#ifndef NDEBUG
  for (auto const &block_entry : block_chain_)
  {
    assert(block_entry_.second.block_number_ > 0 || block_entry_.first == chain::GENSIS_DIGEST);
  }
#endif

  Blocks result;
  bool   proceed = true;

  for (BlockHash current_hash = std::move(start);
       // exit once we have gathered enough blocks or reached genesis
       proceed && result.size() < MainChain::UPPER_BOUND;)
  {
    // look up the block
    auto block = GetBlock(current_hash);
    if (!block)
    {
      FETCH_LOG_ERROR(LOGGING_NAME, "Block lookup failure for block: ", ToBase64(current_hash));
      throw std::runtime_error("Failed to look up block");
    }

    if (block->block_number < lowest_block_number)
    {
      break;
    }

    // walk the hash
    proceed = block->block_number > lowest_block_number;

    if (proceed)
    {
      current_hash = block->previous_hash;
    }

    // update the results
    result.push_back(std::move(block));
  }

  return result;
}

/**
 * Walk the block history collecting blocks until either genesis or the block_number limit reached.
 * limit specifies block_number; if current_hash identifies a block with a higher number
 * travel back in time, otherwise fast forward.
 * If current_hash is empty, travel starts from tip with zero limit, and from genesis otherwise.
 *
 * @param current_hash The hash of the first block
 * @param limit
 * @return The array of blocks
 * @throws std::runtime_error if a block lookup occurs
 */
MainChain::Travelogue MainChain::TimeTravel(BlockHash current_hash, uint64_t limit) const
{
  IntBlockPtr block;
  BlockHash   next_hash;
  FETCH_LOCK(lock_);

  if (current_hash.empty() && limit > 0)
  {
	  current_hash = chain::GENESIS_DIGEST;
  }
  if (!current_hash.empty())
  {
	  if (!LookupBlock(current_hash, block, &next_hash))
	  {
		  FETCH_LOG_ERROR(LOGGING_NAME, "Block lookup failure for block: ", ToBase64(current_hash));
		  throw std::runtime_error("Failed to lookup block");
	  }
  }

  // byt this moment, current hash can only be empty if limit = 0,
  // i.e. travelling back in time from tip
  assert(current_hash.empty() && limit == 0 || block);
  if (current_hash.empty() || limit <= block->block_number)
  {
    // travel back in time
    auto ret_blocks = current_hash.empty() ? GetHeaviestChain(limit)
                                           : GetChainPreceding(std::move(current_hash), limit);
    if (ret_blocks.empty())
    {
      // stop here
      return {Blocks{}, 0};
    }

    auto const earliest_number = ret_blocks.back()->block_number;
    assert(earliest_number >= limit);

    int64_t next_direction = earliest_number == limit ? 0 : -1;
    return {std::move(ret_blocks), next_direction};
  }

  MilliTimer myTimer("MainChain::ChainPreceding");

  Blocks result{block};

  int64_t next_direction = 1;

  for (current_hash = std::move(next_hash);
       // stop once we have gathered enough blocks or passed the tip
       !current_hash.empty() && result.size() < UPPER_BOUND;
       // walk the stack
       current_hash = std::move(next_hash))
  {
    // lookup the block in storage
    if (!LookupBlock(current_hash, block, &next_hash))
    {
      if (IsBlockInCache(current_hash))
      {
        // The block is in the cache yet GetBlock() failed.
        // This indicates that forward reference is ambiguous, so we stop the loop here
        // and the remote requester should now travel from the heaviest tip.
        next_direction = -1;  // signal that forward-travelling can go no further
        break;
      }
      FETCH_LOG_ERROR(LOGGING_NAME, "Block lookup failure for block: ", ToBase64(current_hash));
      throw std::runtime_error("Failed to lookup block");
    }
    // update the results
    result.push_back(std::move(block));
    if (block->block_number >= lim)
    {
      break;
    }
  }
  if (current_hash.empty())
  {
    // we have reached the tip of the chain, stop here
    next_direction = 0;
  }

  return {std::move(result), next_direction};
}

/**
 * Get a common sub tree from the chain.
 *
 * This function will search down both input references input nodes until a common ancestor is
 * found. Once found the blocks from the specified tip that to that common ancestor are returned.
 * Note: untrusted actors should not be allowed to call with the behaviour of returning the oldest
 * block.
 *
 * @param blocks The output list of blocks (from heaviest to lightest)
 * @param tip The tip the output list should start from
 * @param node A node in chain that is searched for
 * @param limit The maximum number of nodes to be returned
 * @param behaviour What to do when the limit is exceeded - either return most or least recent.
 *
 * @return true if successful, otherwise false
 */
bool MainChain::GetPathToCommonAncestor(Blocks &blocks, BlockHash tip, BlockHash node,
                                        uint64_t limit, BehaviourWhenLimit behaviour) const
{
  limit = std::min(limit, uint64_t{MainChain::UPPER_BOUND});
  MilliTimer myTimer("MainChain::GetPathToCommonAncestor", 500);

  FETCH_LOCK(lock_);

  bool success{true};

  // clear the output structure
  blocks.clear();

  BlockPtr left{};
  BlockPtr right{};

  BlockHash left_hash  = std::move(tip);
  BlockHash right_hash = std::move(node);

  std::deque<BlockPtr> res;

  // The algorithm implemented here is effectively a coordinated parallel walk about from the two
  // input tips until the a common ancestor is located.
  for (;;)
  {
    // load up the left side
    if (!left || left->hash != left_hash)
    {
      left = GetBlock(left_hash);
      if (!left)
      {
        FETCH_LOG_WARN(LOGGING_NAME, "Unable to look up block (left): ", ToBase64(left_hash));
        success = false;
        break;
      }

      // left side always loaded into output queue as we traverse
      // blocks.push_back(left);
      res.push_back(left);
      bool break_loop = false;

      switch (behaviour)
      {
      case BehaviourWhenLimit::RETURN_LEAST_RECENT:
        if (res.size() > limit)
        {
          res.pop_front();
        }
        break;
      case BehaviourWhenLimit::RETURN_MOST_RECENT:
        if (res.size() >= limit)
        {
          break_loop = true;
        }
        break;
      }

      if (break_loop)
      {
        break;
      }
    }

    // load up the right side
    if (!right || right->hash != right_hash)
    {
      right = GetBlock(right_hash);
      if (!right)
      {
        FETCH_LOG_WARN(LOGGING_NAME, "Unable to look up block (right): ", ToBase64(right_hash));
        success = false;
        break;
      }
    }

    FETCH_LOG_DEBUG(LOGGING_NAME, "Left: ", ToBase64(left_hash), " -> ", left->block_number,
                    " Right: ", ToBase64(right_hash), " -> ", right->block_number);

    if (left_hash == right_hash)
    {
      break;
    }

    if (left->block_number <= right->block_number)
    {
      right_hash = right->previous_hash;
    }

    if (left->block_number >= right->block_number)
    {
      left_hash = left->previous_hash;
    }
  }

  blocks.resize(res.size());
  std::move(res.begin(), res.end(), blocks.begin());

  // If a lookup error has occurred then we do not return anything
  if (!success)
  {
    blocks.clear();
  }

  return success;
}

/**
 * Retrieve a block with a specific hash
 *
 * @param hash The hash being queried
 * @return The a valid shared pointer to the block if found, otherwise an empty pointer
 */
MainChain::BlockPtr MainChain::GetBlock(BlockHash const &hash) const
{
  FETCH_LOCK(lock_);

  BlockPtr output_block{};

  // attempt to look up the block
  auto internal_block = std::make_shared<Block>();
  if (LookupBlock(hash, internal_block))
  {
    // convert the pointer type to per const
    output_block = std::static_pointer_cast<Block const>(internal_block);
  }
  else
  {
    FETCH_LOG_WARN(LOGGING_NAME, "main chain failed to look up block! Hash: ", hash.ToHex());
  }

  return output_block;
}

/**
 * Return a copy of the missing block tips
 *
 * @return A set of tip hashes
 */
MainChain::BlockHashSet MainChain::GetMissingTips() const
{
  FETCH_LOCK(lock_);

  BlockHashSet tips{};
  for (auto const &element : loose_blocks_)
  {
    // tips are blocks that are referenced but we have not seen them yet. Since all loose blocks are
    // stored in the cache, we simply evaluate which of the loose references we do not currently
    // have in the cache
    if (!IsBlockInCache(element.first))
    {
      tips.insert(element.first);
    }
  }

  return tips;
}

/**
 * Retrieve an array of all the missing block hashes
 *
 * @param maximum The specified maximum number of blocks to be returned
 * @return The generated array of missing hashes
 */
MainChain::BlockHashes MainChain::GetMissingBlockHashes(uint64_t limit) const
{
  limit = std::min(limit, uint64_t{MainChain::UPPER_BOUND});
  FETCH_LOCK(lock_);

  BlockHashes results;

  for (auto const &loose_block : loose_blocks_)
  {
    if (limit <= results.size())
    {
      break;
    }
    results.push_back(loose_block.first);
  }

  return results;
}

/**
 * Determine if the chain has missing blocks
 *
 * @return true if the chain is missing blocks, otherwise false
 */
bool MainChain::HasMissingBlocks() const
{
  FETCH_LOCK(lock_);
  return !loose_blocks_.empty();
}

/**
 * Get the set of all the tips
 *
 * @return The set of tip hashes
 */
MainChain::BlockHashSet MainChain::GetTips() const
{
  FETCH_LOCK(lock_);

  BlockHashSet hash_set;
  for (auto const &element : tips_)
  {
    hash_set.insert(element.first);
  }

  return hash_set;
}

/**
 * Internal: Initialise and recover all previous state from the chain
 *
 * @param mode The storage mode for the chain
 */
void MainChain::RecoverFromFile(Mode mode)
{
  // TODO(private issue 667): Complete loading of chain on startup ill-advised

  assert(static_cast<bool>(block_store_));

  FETCH_LOCK(lock_);

  // load the database files
  if (Mode::CREATE_PERSISTENT_DB == mode)
  {
    block_store_->New("chain.db", "chain.index.db");
    head_store_.open("chain.head.db",
                     std::ios::binary | std::ios::in | std::ios::out | std::ios::trunc);
    return;
  }
  assert(mode == Mode::LOAD_PERSISTENT_DB);
  if (Mode::LOAD_PERSISTENT_DB == mode)
  {
    block_store_->Load("chain.db", "chain.index.db");
    head_store_.open("chain.head.db", std::ios::binary | std::ios::in | std::ios::out);
  }

  // load the head block, and attempt verify that this block forms a complete chain to genesis
  IntBlockPtr head = std::make_shared<Block>();

  // retrieve the starting hash
  BlockHash head_block_hash = GetHeadHash();

  bool recovery_complete{false};
  if (!head_block_hash.empty() && LoadBlock(head_block_hash, *head))
  {
    auto block_index = head->block_number;

    // Copy head block so as to walk down the chain
    IntBlockPtr next = std::make_shared<Block>(*head);

    while (LoadBlock(next->previous_hash, *next))
    {
      if (next->block_number != block_index - 1)
      {
        FETCH_LOG_WARN(LOGGING_NAME,
                       "Discontinuity found when walking main chain during recovery. Current: ",
                       block_index, " prev: ", next->block_number, " Resetting");
        break;
      }

      block_index = next->block_number;
    }

    if (block_index != 0)
    {
      FETCH_LOG_WARN(LOGGING_NAME,
                     "Failed to walk main chain when recovering from disk. Got as far back as: ",
                     block_index, ". Resetting.");
    }
    else
    {
      FETCH_LOG_INFO(LOGGING_NAME,
                     "Recovering main chain with heaviest block: ", head->block_number);

      // Add heaviest to cache
      CacheBlock(head);

      // Update this as our heaviest
      bool const result = heaviest_.Update(*head);
      tips_[head->hash] = Tip{head->total_weight, head->weight, head->block_number};

      if (!result)
      {
        FETCH_LOG_WARN(LOGGING_NAME, "Failed to update heaviest when loading from file.");
      }

      // Sanity check
      uint64_t heaviest_block_num = GetHeaviestBlock()->block_number;
      FETCH_LOG_INFO(LOGGING_NAME, "Heaviest block: ", heaviest_block_num);

      DetermineHeaviestTip();
      heaviest_block_num = GetHeaviestBlock()->block_number;
      FETCH_LOG_INFO(LOGGING_NAME, "Heaviest block now: ", heaviest_block_num);
      FETCH_LOG_INFO(LOGGING_NAME, "Heaviest block weight: ", GetHeaviestBlock()->total_weight);

      // signal that the recovery was successful
      recovery_complete = true;
    }
  }
  else
  {
    FETCH_LOG_INFO(LOGGING_NAME,
                   "No head block found in chain data store! Resetting chain data store.");
  }

  // Recovering the chain has failed in some way, reset the storage.
  if (!recovery_complete)
  {
    block_store_->New("chain.db", "chain.index.db");

    // reopen the file and clear the contents
    head_store_.close();
    head_store_.open("chain.head.db",
                     std::ios::binary | std::ios::in | std::ios::out | std::ios::trunc);
  }
}

/**
 * Internal: Flush confirmed blocks to disk
 */
void MainChain::WriteToFile()
{
  // look up the heaviest block
  IntBlockPtr block = block_chain_.at(heaviest_.hash);

  // skip if the block store is not persistent
  if (block_store_ && (block->block_number >= chain::FINALITY_PERIOD))
  {
    MilliTimer myTimer("MainChain::WriteToFile", 500);

    // Add confirmed blocks to file, minus finality

    // Find the block N back from our heaviest
    bool failed = false;
    for (std::size_t i = 0; i < chain::FINALITY_PERIOD; ++i)
    {
      if (!LookupBlock(block->previous_hash, block))
      {
        failed = true;
        break;
      }
    }

    if (failed)
    {
      FETCH_LOG_WARN(LOGGING_NAME,
                     "Failed to walk back the chain when writing to file! Block head: ",
                     block_chain_.at(heaviest_.hash)->block_number);
      return;
    }

    // This block will now become the head in our file
    // Corner case - block is genesis
    if (block->IsGenesis())
    {
      FETCH_LOG_DEBUG(LOGGING_NAME, "Writing genesis. ");

      KeepBlock(block);
      SetHeadHash(block->hash);
    }
    else
    {
      FETCH_LOG_DEBUG(LOGGING_NAME, "Writing block. ", block->block_number);

      // Recover the current head block from the file
      IntBlockPtr current_file_head = std::make_shared<Block>();
      IntBlockPtr block_head        = block;

      LoadBlock(GetHeadHash(), *current_file_head);

      // Now keep adding the block and its prev to the file until we are certain the file contains
      // an unbroken chain. Assuming that the current_file_head is unbroken we can write until we
      // touch it or it's root.
      for (;;)
      {
        KeepBlock(block);

        // Keep the current_file_head one block behind
        while (current_file_head->block_number > block->block_number - 1)
        {
          LoadBlock(current_file_head->previous_hash, *current_file_head);
        }

        // Successful case
        if (current_file_head->hash == block->previous_hash)
        {
          break;
        }

        // Continue to push previous into file
        LookupBlock(block->previous_hash, block);
      }

      // Success - we kept a copy of the new head to write
      SetHeadHash(block_head->hash);
    }

    // Clear the block from ram
    FlushBlock(block);

    // Force flush of the file object!
    block_store_->Flush(false);

    // as final step do some sanity checks
    TrimCache();
  }
}

/**
 * Trim the in memory cache
 *
 * Only should be called if the block store is being used
 */
void MainChain::TrimCache()
{
  static const uint64_t CACHE_TRIM_THRESHOLD = 2 * chain::FINALITY_PERIOD;
  assert(static_cast<bool>(block_store_));

  MilliTimer myTimer("MainChain::TrimCache");

  FETCH_LOCK(lock_);

  uint64_t const heaviest_block_num = GetHeaviestBlock()->block_number;

  if (CACHE_TRIM_THRESHOLD < heaviest_block_num)
  {
    uint64_t const trim_threshold = heaviest_block_num - CACHE_TRIM_THRESHOLD;

    // Loop through the block chain store looking for blocks which are outside of our finality
    // period. This is needed to ensure that the block chain map does not grow forever
    auto chain_it = block_chain_.begin();
    while (chain_it != block_chain_.end())
    {
      auto const &block = chain_it->second;

      if (trim_threshold >= block->block_number)
      {
        FETCH_LOG_INFO(LOGGING_NAME, "Removing loose block: 0x", block->hash.ToHex());

        // remove the entry from the tips map
        tips_.erase(block->hash);

        // remove any reference in the loose map
        auto loose_it = loose_blocks_.find(block->previous_hash);
        if (loose_it != loose_blocks_.end())
        {
          // remove the hash from the list
          loose_it->second.remove(block->hash);

          // if, as a result, the list is empty then also remove the entry from the loose blocks
          if (loose_it->second.empty())
          {
            loose_blocks_.erase(loose_it);
          }
        }

        // remove the entry from the main block chain
        chain_it = block_chain_.erase(chain_it);
      }
      else
      {
        // normal case advance to the next one
        ++chain_it;
      }
    }
  }

  // Debug and sanity check
  auto loose_it = loose_blocks_.begin();
  while (loose_it != loose_blocks_.end())
  {
    FETCH_LOG_DEBUG(LOGGING_NAME, "Cleaning loose map entry: ", loose_it->first.ToBase64());

    if (loose_it->second.empty())
    {
      loose_it = loose_blocks_.erase(loose_it);
    }
    else
    {
      ++loose_it;
    }
  }
}

/**
 * Internal: Remove the block from the cache and remove any tips
 *
 * @param block The block to be flushed
 */
void MainChain::FlushBlock(IntBlockPtr const &block)
{
  // remove the block from the block map
  UncacheBlock(block->hash);

  // remove the block hash from the tips
  tips_.erase(block->hash);
}

// We have added a non-loose block. It is then safe to lock the loose blocks map and
// walk through it adding the blocks, so long as we do breadth first search (!!)
void MainChain::CompleteLooseBlocks(IntBlockPtr const &block)
{
  FETCH_LOCK(lock_);

  // Determine if this block is actually a loose block, if it isn't exit immediately
  auto it = loose_blocks_.find(block->hash);
  if (it == loose_blocks_.end())
  {
    return;
  }

  // At this point we are sure that the current block is one of the missing blocks we are after

  // Extract the list of blocks that are waiting on this block
  BlockHashList blocks_to_add = std::move(it->second);
  loose_blocks_.erase(it);

  FETCH_LOG_DEBUG(LOGGING_NAME, blocks_to_add.size(), " are resolved from 0x", block->hash.ToHex());

  while (!blocks_to_add.empty())
  {
    BlockHashList next_blocks_to_add{};

    // This is the breadth-first search, for each block to add, add it, next_blocks_to_add will
    // get pushed with the next layer of blocks
    for (auto const &hash : blocks_to_add)
    {
      // This should be guaranteed safe
      IntBlockPtr add_block = block_chain_.at(hash);  // TODO(EJF): What happens when this fails

      // This won't re-call this function due to the flag
      InsertBlock(add_block, false);

      // The added block was not loose. Continue to clear
      auto const it2 = loose_blocks_.find(add_block->hash);
      if (it2 != loose_blocks_.end())
      {
        // add all the items to the next list
        next_blocks_to_add.splice(next_blocks_to_add.end(), it2->second);

        // remove the list from the blocks list
        loose_blocks_.erase(it2);
      }
    }

    blocks_to_add = std::move(next_blocks_to_add);
  }
}

/**
 * Records the input block as loose
 *
 * @param block The reference to the block being marked as loose
 */
void MainChain::RecordLooseBlock(IntBlockPtr const &block)
{
  FETCH_LOCK(lock_);

  // Get vector of waiting blocks and push ours on
  auto &waiting_blocks = loose_blocks_[block->previous_hash];
  waiting_blocks.push_back(block->hash);

  assert(block->is_loose);
  CacheBlock(block);
}

/**
 * Update tips (including the heaviest) if necessary
 *
 * @param block The current block
 * @param prev_block The previous block
 * @return true if the heaviest tip was advanced, otherwise false
 */
bool MainChain::UpdateTips(IntBlockPtr const &block)
{
  assert(!block->is_loose);
  assert(block->weight != 0);
  assert(block->total_weight != 0);
  assert(block->block_number != 0);

  // remove the tip if exists and add the new one
  tips_.erase(block->previous_hash);
  tips_[block->hash] = Tip{block->total_weight, block->weight, block->block_number};

  // attempt to update the heaviest tip
  return heaviest_.Update(*block);
}

/**
 * Internal: Insert the block into the cache
 *
 * @param block The block to be inserted
 * @param evaluate_loose_blocks Flag to signal if the loose blocks should be evaluated
 * @return
 */
BlockStatus MainChain::InsertBlock(IntBlockPtr const &block, bool evaluate_loose_blocks)
{
  assert(!block->previous_hash.empty());

  MilliTimer myTimer("MainChain::InsertBlock", 500);

  FETCH_LOCK(lock_);

  if (block->hash.empty())
  {
    FETCH_LOG_WARN(LOGGING_NAME, "Block discard due to lack of digest");
    return BlockStatus::INVALID;
  }

  if (block->hash == block->previous_hash)
  {
    FETCH_LOG_WARN(LOGGING_NAME, "Block discard due to invalid digests");
    return BlockStatus::INVALID;
  }

  // Assume for the moment that this block is not loose. The validity of this statement will be
  // checked below
  block->is_loose = false;

  IntBlockPtr prev_block{};
  if (evaluate_loose_blocks)  // normal case - not being called from inside CompleteLooseBlocks
  {
    // First check if block already exists (not checking in object store)
    if (IsBlockInCache(block->hash))
    {
      FETCH_LOG_DEBUG(LOGGING_NAME, "Attempting to add already seen block");
      return BlockStatus::DUPLICATE;
    }

    // Determine if the block is present in the cache
    if (LookupBlock(block->previous_hash, prev_block))
    {
      // TODO(EJF): Add check to validate the block number (it is relied on heavily now)
      if (block->block_number != (prev_block->block_number + 1))
      {
        FETCH_LOG_INFO(LOGGING_NAME, "Block 0x", block->hash.ToHex(), " has invalid block number");
        return BlockStatus::INVALID;
      }

      // Also a loose block if it points to a loose block
      if (prev_block->is_loose)
      {
        // since we are connecting to loose block, by definition this block is also loose
        block->is_loose = true;

        FETCH_LOG_DEBUG(LOGGING_NAME, "Block connects to loose block");
      }
    }
    else
    {
      // This is the normal case where we do not have a previous hash
      block->is_loose = true;

      FETCH_LOG_DEBUG(LOGGING_NAME,
                      "Previous block not found: ", byte_array::ToBase64(block->previous_hash));
    }
  }
  else  // special case - being called from inside CompleteLooseBlocks
  {
    // This branch is a small optimisation since loose / missing blocks are not flushed to disk

    if (!LookupBlockFromCache(block->previous_hash, prev_block))
    {
      // This is currently only called from inside CompleteLooseBlocks and it is invariant on the
      // return value. For completeness this block is however loose because the parent can not be
      // located.
      return BlockStatus::LOOSE;
    }
  }

  if (block->is_loose)
  {
    // record the block as loose
    RecordLooseBlock(block);
    return BlockStatus::LOOSE;
  }

  // we expect only non-loose blocks here
  assert(!block->is_loose);

  // by definition this also means we expect blocks to have a valid parent block too
  assert(static_cast<bool>(prev_block));

  // update the final (total) weight for this block
  block->total_weight = prev_block->total_weight + block->weight;

  // At this point we can proceed knowing that the block is building upon existing tip

  // At this point we have a new block with a prev that's known and not loose. Update tips
  bool const heaviest_advanced = UpdateTips(block);

  // Add block
  FETCH_LOG_DEBUG(LOGGING_NAME, "Adding block to chain: 0x", block->hash.ToHex());
  AddBlockToCache(block);

  // If the heaviest branch has been updated we should determine if any blocks should be flushed
  // to disk
  if (heaviest_advanced)
  {
    WriteToFile();
  }

  // Now we're done, it's possible this added block completed some loose blocks.
  if (evaluate_loose_blocks)
  {
    CompleteLooseBlocks(block);
  }

  return BlockStatus::ADDED;
}

/**
 * Attempt to look up a block.
 *
 * The search is performed initially on the in memory cache and then if this fails the persistent
 * disk storage is searched
 *
 * @param hash The hash of the block to search for
 * @param block The output block to be populated
 * @param next_hash If non-null, next_hash of this block is to be copied to that location
 *
 * @return true if successful, otherwise false
 */
bool MainChain::LookupBlock(BlockHash const &hash, IntBlockPtr &block, BlockHash *next_hash) const
{
  if (LookupBlockFromCache(hash, block))
  {
    if (next_hash == nullptr                   // either forward reference is not needed
        || LookupReference(hash, *next_hash))  // or forward reference is required and unique
    {
      return true;
    }
  }
  // either block is not in the cache, or forward reference cannot be resolved unambiguously
  return LookupBlockFromStorage(hash, block, next_hash);
}

/**
 * Attempt to locate a block stored in the im memory cache
 *
 * @param hash The hash of the block to search for
 * @param block The output block to be populated
 * @return true if successful, otherwise false
 */
bool MainChain::LookupBlockFromCache(BlockHash const &hash, IntBlockPtr &block) const
{
  FETCH_LOCK(lock_);

  // perform the lookup
  auto const it = block_chain_.find(hash);
  if ((block_chain_.end() != it))
  {
    block = it->second;
    return true;
  }

  return false;
}

/**
 * Attempt to locate a block stored in the persistent object store
 *
 * @param hash The hash of the block to search for
 * @param block The output block to be populated
 * @return true if successful, otherwise false
 */
bool MainChain::LookupBlockFromStorage(BlockHash const &hash, IntBlockPtr &block,
                                       BlockHash *next_hash) const
{
  bool success{false};

  if (block_store_)
  {
    // create the output block
    auto output_block = std::make_shared<Block>();

    // attempt to read the block from the storage engine
    success = LoadBlock(hash, *output_block, next_hash);

    if (success)
    {
      // hash not serialised, needs to be recomputed
      output_block->UpdateDigest();

      // update the returned shared pointer
      block = output_block;
    }
  }

  return success;
}

/**
 * No Locking: Determine is a specified block is in the cache
 *
 * @param hash The hash to query
 * @return true if the block is present, otherwise false
 */
bool MainChain::IsBlockInCache(BlockHash const &hash) const
{
  return block_chain_.find(hash) != block_chain_.end();
}

/**
 * Add the block to the cache as a non-loose block
 *
 * @param block The block to be written into the cache
 */
void MainChain::AddBlockToCache(IntBlockPtr const &block) const
{
  block->is_loose = false;

  if (!IsBlockInCache(block->hash))
  {
    // add the item to the block chain storage
    CacheBlock(block);
  }
}

/**
 * Adds a new tip for the given block
 *
 * @param block The block for the tip to be created from
 */
bool MainChain::AddTip(IntBlockPtr const &block)
{
  FETCH_LOCK(lock_);

  // record the tip weight
  tips_[block->hash] = Tip{block->total_weight, block->weight, block->block_number};

  return DetermineHeaviestTip();
}

/**
 * Evaluate the current set of tips in the chain and determine which of them is the heaviest
 *
 * @return true if successful, otherwise false
 */
bool MainChain::DetermineHeaviestTip()
{
  FETCH_LOCK(lock_);

  if (!tips_.empty())
  {
    // find the heaviest item in our tip selection
    auto it = std::max_element(
        tips_.begin(), tips_.end(), [](TipsMap::value_type const &a, TipsMap::value_type const &b) {
          auto        a_total_weight{a.second.total_weight}, b_total_weight{b.second.total_weight};
          auto const &a_hash{a.first}, &b_hash{b.first};
          auto        a_weight{a.second.weight}, b_weight{b.second.weight};
          auto        a_height{a.second.block_number}, b_height{b.second.block_number};

          // Tips are selected based on the following priority of properties:
          // 1. total weight
          // 2. block number (long chain)
          // 3. weight, which is related to the rank of the miner producing the block
          // 4. hash - note this case should never be required if stutter blocks are removed from
          // tips
          //
          // Chains of equivalent total weight and length are tie-broken, choosing the weight of the
          // tips as a tiebreaker. This is important for consensus.
          return HeaviestTip::TipStats{a_total_weight, a_height, a_weight, a_hash} <
                 HeaviestTip::TipStats{b_total_weight, b_height, b_weight, b_hash};
        });

    // update the heaviest
    heaviest_.hash         = it->first;
    heaviest_.weight       = it->second.weight;
    heaviest_.total_weight = it->second.total_weight;
    heaviest_.block_number = it->second.block_number;
    return true;
  }

  return false;
}

/**
 * Reindex the tips
 *
 * This is a currently an expensive operation which will scale in the order N(1 + logN) in the worst
 * case.
 *
 * @return true if successful, otherwise false
 */
bool MainChain::ReindexTips()
{
  FETCH_LOCK(lock_);

  // Tips are hashes of cached non-loose blocks that don't have any forward references
  TipsMap   new_tips;
  uint64_t  max_total_weight{};
  uint64_t  max_weight{};
  uint64_t  max_block_number{};
  BlockHash max_hash;

  for (auto const &block_entry : block_chain_)
  {
    if (block_entry.second->is_loose)
    {
      continue;
    }
    auto const &hash{block_entry.first};
    // check if this has has any live forward reference
    auto children{forward_references_.equal_range(hash)};
    auto child{std::find_if(children.first, children.second, [this](auto const &ref) {
      return block_chain_.find(ref.second) != block_chain_.end();
    })};
    if (child != children.second)
    {
      // then it's not a tip
      continue;
    }
    // this hash has no next blocks
    auto const &   block{*block_entry.second};
    const uint64_t total_weight{block.total_weight};
    const uint64_t weight{block.weight};
    const uint64_t block_number{block.block_number};
    new_tips[hash] = Tip{total_weight, weight, block_number};
    // check if this tip is the current heaviest
    if (HeaviestTip::TipStats{total_weight, block_number, weight, hash} >
        HeaviestTip::TipStats{max_total_weight, max_block_number, max_weight, max_hash})
    {
      max_total_weight = total_weight;
      max_weight       = weight;
      max_hash         = hash;
      max_block_number = block_number;
    }
  }
  tips_ = std::move(new_tips);

  if (!tips_.empty())
  {
    // finally update the heaviest tip
    heaviest_.total_weight = max_total_weight;
    heaviest_.weight       = max_weight;
    heaviest_.hash         = max_hash;
    heaviest_.block_number = max_block_number;
    return true;
  }

  return false;
}

/**
 * Create the initial genesis block
 * @return The generated block
 */
MainChain::IntBlockPtr MainChain::CreateGenesisBlock()
{
  auto genesis           = std::make_shared<Block>();
  genesis->previous_hash = chain::ZERO_HASH;
  genesis->hash          = chain::GENESIS_DIGEST;
  genesis->merkle_hash   = chain::GENESIS_MERKLE_ROOT;
  genesis->miner         = chain::Address{crypto::Hash<crypto::SHA256>("")};
  genesis->is_loose      = false;

  return genesis;
}

/**
 * Gets the current hash of the heaviest chain
 *
 * @return The heaviest chain hash
 */
MainChain::BlockHash MainChain::GetHeaviestBlockHash() const
{
  FETCH_LOCK(lock_);
  return heaviest_.hash;
}

/**
 * Determines if the new block is the heaviest and if so updates its internal state
 *
 * @param block The candidate block being evaluated
 * @return true if the heaviest tip was updated, otherwise false
 */
bool MainChain::HeaviestTip::Update(Block const &block)
{
  if (TipStats{block.total_weight, block.block_number, block.weight, block.hash} > Stats())
  {
    FETCH_LOG_DEBUG(LOGGING_NAME, "New heaviest tip: 0x", block.hash.ToHex());

    total_weight = block.total_weight;
    weight       = block.weight;
    hash         = block.hash;
    block_number = block.block_number;
    return true;
  }

  return false;
}

MainChain::BlockHash MainChain::GetHeadHash()
{
  byte_array::ByteArray buffer;

  // determine is the hash has already been stored once
  head_store_.seekg(0, std::ios::end);
  auto const file_size = head_store_.tellg();

  if (file_size == 32)
  {
    buffer.Resize(32);

    // return to the beginning and overwrite the hash
    head_store_.seekg(0);
    head_store_.read(reinterpret_cast<char *>(buffer.pointer()),
                     static_cast<std::streamsize>(buffer.size()));
  }

  return {buffer};
}

void MainChain::SetHeadHash(BlockHash const &hash)
{
  assert(hash.size() == chain::HASH_SIZE);

  // move to the beginning of the file and write out the hash
  head_store_.seekp(0);
  head_store_.write(reinterpret_cast<char const *>(hash.pointer()),
                    static_cast<std::streamsize>(hash.size()));
}

/**
 * Strip transactions in container that already exist in the blockchain
 *
 * @param: starting_hash Block to start looking downwards from
 * @tparam: transaction The set of transaction to be filtered
 *
 * @return: bool whether the starting hash referred to a valid block on a valid chain
 */
DigestSet MainChain::DetectDuplicateTransactions(BlockHash const &starting_hash,
                                                 DigestSet const &transactions) const
{
  MilliTimer const timer{"DuplicateTransactionsCheck", 100};

  FETCH_LOG_DEBUG(LOGGING_NAME, "Starting TX uniqueness verify");

  IntBlockPtr block;
  if (!LookupBlock(starting_hash, block) || block->is_loose)
  {
    FETCH_LOG_WARN(LOGGING_NAME, "TX uniqueness verify on bad block hash");
    return {};
  }

  DigestSet potential_duplicates{};
  for (auto const &digest : transactions)
  {

    std::pair<bool, std::size_t> const result = bloom_filter_->Match(digest);
    bloom_filter_queried_bit_count_->set(result.second);
    if (result.first)
    {
      bloom_filter_positive_count_->increment();
      potential_duplicates.insert(digest);
    }
    bloom_filter_query_count_->increment();
  }

  auto search_chain_for_duplicates =
      [this, block](DigestSet const &transaction_digests) mutable -> DigestSet {
    DigestSet duplicates{};
    for (;;)
    {
      // Traversing the chain fully is costly: break out early if we know the transactions are all
      // duplicated (or both sets are empty)
      if (transaction_digests.size() == duplicates.size())
      {
        break;
      }

      for (auto const &slice : block->slices)
      {
        for (auto const &tx : slice)
        {
          if (transaction_digests.find(tx.digest()) != transaction_digests.end())
          {
            duplicates.insert(tx.digest());
          }
        }
      }

      // exit the loop once we can no longer find the block
      if (!LookupBlock(block->previous_hash, block))
      {
        break;
      }
    }

    return duplicates;
  };

  DigestSet const duplicates =
      search_chain_for_duplicates(enable_bloom_filter_ ? potential_duplicates : transactions);

  auto const false_positives = potential_duplicates.size() - duplicates.size();

  bloom_filter_false_positive_count_->add(false_positives);

  if (bloom_filter_->ReportFalsePositives(false_positives))
  {
    FETCH_LOG_INFO(LOGGING_NAME, "Bloom filter false positive rate exceeded threshold");
  }

  return duplicates;
}

constexpr char const *ToString(BlockStatus status)
{
  switch (status)
  {
  case BlockStatus::ADDED:
    return "Added";
  case BlockStatus::LOOSE:
    return "Loose";
  case BlockStatus::DUPLICATE:
    return "Duplicate";
  case BlockStatus::INVALID:
    return "Invalid";
  default:
    return "Unknown";
  }
}

}  // namespace ledger
}  // namespace fetch
