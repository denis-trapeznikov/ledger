#pragma once

#include "oef-base/threading/Waitable.hpp"
#include <atomic>

template <class T, typename RESULT_TYPE>
class FutureCombiner : public Waitable,
                       public std::enable_shared_from_this<FutureCombiner<T, RESULT_TYPE>>
{
public:
  static constexpr char const *LOGGING_NAME = "FutureCombiner";

  using ResultMerger =
      std::function<void(std::shared_ptr<RESULT_TYPE> &, const std::shared_ptr<RESULT_TYPE> &)>;

  FutureCombiner()
    : Waitable()
    , result_{std::make_shared<RESULT_TYPE>()}
    , completed_{0}
  {}

  virtual ~FutureCombiner()
  {}

  void AddFuture(std::shared_ptr<T> future)
  {
    auto                                          this_sp = this->shared_from_this();
    std::weak_ptr<FutureCombiner<T, RESULT_TYPE>> this_wp = this_sp;

    uint32_t future_idx = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);

      future_idx = futures_.size();
      futures_.push_back(future);
    }

    FETCH_LOG_INFO(LOGGING_NAME, "Added future: idx=", future_idx);

    future->makeNotification().Then([this_wp, future_idx]() {
      FETCH_LOG_INFO(LOGGING_NAME, "Got future! idx = ", future_idx);
      auto sp = this_wp.lock();
      if (sp)
      {
        try
        {
          std::lock_guard<std::mutex> lock(sp->mutex_);
          auto                        res_future = sp->futures_.at(future_idx)->get();
          if (res_future)
          {
            sp->future_combiner_(sp->result_, res_future);
          }
          else
          {
            FETCH_LOG_WARN(LOGGING_NAME, "Got nullptr from future: ", future_idx);
          }
          ++(sp->completed_);
          if (sp->futures_.size() == sp->completed_)
          {
            sp->wake();
            sp->futures_.clear();
          }
        }
        catch (std::exception &e)
        {
          FETCH_LOG_WARN(LOGGING_NAME, "Exception while processing new future result: ", e.what());
        }
      }
      else
      {
        FETCH_LOG_WARN(LOGGING_NAME, "Failed to lock weak ptr! Future ignored!");
      }
    });
  }

  void SetResultMerger(ResultMerger futureCombiner)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    future_combiner_ = std::move(futureCombiner);
  }

  std::shared_ptr<RESULT_TYPE> &Get()
  {
    return result_;
  }

protected:
  std::mutex                      mutex_{};
  std::vector<std::shared_ptr<T>> futures_{};
  std::shared_ptr<RESULT_TYPE>    result_;
  ResultMerger                    future_combiner_;
  uint32_t                        completed_;

private:
  FutureCombiner(const FutureCombiner &other) = delete;  // { copy(other); }
  FutureCombiner &operator                    =(const FutureCombiner &other) =
      delete;                                             // { copy(other); return *this; }
  bool operator==(const FutureCombiner &other) = delete;  // const { return compare(other)==0; }
  bool operator<(const FutureCombiner &other)  = delete;  // const { return compare(other)==-1; }
};
