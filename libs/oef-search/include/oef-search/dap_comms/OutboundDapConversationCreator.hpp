#pragma once
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

#include <map>
#include <memory>

#include "oef-base/conversation/IOutboundConversationCreator.hpp"
#include "oef-base/conversation/OutboundConversations.hpp"
#include "oef-base/utils/Uri.hpp"

class OutboundConversationWorkerTask;
template <typename TXType, typename Reader, typename Sender>
class ProtoMessageEndpoint;
class Core;

class OutboundDapConversationCreator : public IOutboundConversationCreator
{
public:
  OutboundDapConversationCreator(size_t thread_group_id, const Uri &dap_uri, Core &core,
                                 std::shared_ptr<OutboundConversations> outbounds);
  virtual ~OutboundDapConversationCreator();
  virtual std::shared_ptr<OutboundConversation> start(
      const Uri &target_path, std::shared_ptr<google::protobuf::Message> initiator);

protected:
private:
  static constexpr char const *LOGGING_NAME = "OutboundDapConversationCreator";
  using TXType = std::pair<Uri, std::shared_ptr<google::protobuf::Message>>;

  std::size_t ident = 1;

  std::shared_ptr<OutboundConversationWorkerTask> worker;

  std::map<unsigned long, std::shared_ptr<OutboundConversation>> ident2conversation;

  OutboundDapConversationCreator(const OutboundDapConversationCreator &other) = delete;
  OutboundDapConversationCreator &operator=(const OutboundDapConversationCreator &other)  = delete;
  bool                            operator==(const OutboundDapConversationCreator &other) = delete;
  bool                            operator<(const OutboundDapConversationCreator &other)  = delete;
};