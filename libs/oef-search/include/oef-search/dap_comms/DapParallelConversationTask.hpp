#pragma once

#include <memory>
#include <mutex>
#include <queue>
#include <utility>

#include "logging/logging.hpp"
#include "oef-base/threading/TaskChainParallel.hpp"

#include "oef-base/conversation/OutboundConversations.hpp"
#include "oef-search/dap_comms/DapConversationTask.hpp"

template <typename IN_PROTO>
struct DapInputDataType
{
  std::string               dap_name;
  std::string               path;
  std::shared_ptr<IN_PROTO> proto = nullptr;
};

template <typename IN_PROTO, typename OUT_PROTO>
class DapParallelConversationTask
  : virtual public TaskChainParallel<IN_PROTO, OUT_PROTO, DapInputDataType<IN_PROTO>,
                                     DapConversationTask<IN_PROTO, OUT_PROTO>>
{
public:
  using TaskType = DapConversationTask<IN_PROTO, OUT_PROTO>;
  using Parent   = TaskChainParallel<IN_PROTO, OUT_PROTO, DapInputDataType<IN_PROTO>,
                                   DapConversationTask<IN_PROTO, OUT_PROTO>>;
  static constexpr char const *LOGGING_NAME = "DapParallelConversationTask";

  DapParallelConversationTask(uint32_t msg_id, std::shared_ptr<OutboundConversations> outbounds,
                              std::string protocol = "dap")
    : Parent::Parent()
    , Parent()
    , msg_id_{msg_id - 1}
    , outbounds(std::move(outbounds))
    , protocol_{std::move(protocol)}

  {
    FETCH_LOG_INFO(LOGGING_NAME, "Task created.");
  }

  virtual ~DapParallelConversationTask()
  {
    FETCH_LOG_INFO(LOGGING_NAME, "Task gone.");
  }

  DapParallelConversationTask(const DapParallelConversationTask &other) = delete;
  DapParallelConversationTask &operator=(const DapParallelConversationTask &other) = delete;

  bool operator==(const DapParallelConversationTask &other) = delete;
  bool operator<(const DapParallelConversationTask &other)  = delete;

  virtual std::shared_ptr<TaskType> CreateTask(const DapInputDataType<IN_PROTO> &data,
                                               std::shared_ptr<IN_PROTO>         input) override
  {
    return std::make_shared<DapConversationTask<IN_PROTO, OUT_PROTO>>(
        data.dap_name, data.path, ++msg_id_, input, outbounds, protocol_);
  }

  virtual std::shared_ptr<IN_PROTO> GetInputProto(const DapInputDataType<IN_PROTO> &data) override
  {
    return data.proto;
  }

  virtual void Add(DapInputDataType<IN_PROTO> data) override
  {
    idx_to_dap_[this->tasks_.size()] = data.dap_name;
    Parent ::Add(std::move(data));
  }

  const std::string &GetDapName(std::size_t idx) const
  {
    auto it = idx_to_dap_.find(idx);
    if (it != idx_to_dap_.end())
    {
      return it->second;
    }
    return empty_string;
  }

protected:
  uint32_t                                     msg_id_;
  std::shared_ptr<OutboundConversations>       outbounds;
  std::unordered_map<std::size_t, std::string> idx_to_dap_{};
  std::string                                  empty_string{""};
  std::string                                  protocol_;
};
