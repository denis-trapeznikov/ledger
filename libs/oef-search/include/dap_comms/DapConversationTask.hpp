#pragma once

#include <memory>
#include <utility>

#include "base/src/cpp/threading/StateMachineTask.hpp"
#include "fetch_teams/ledger/logger.hpp"
#include "base/src/cpp/conversation/OutboundConversations.hpp"
#include "base/src/cpp/conversation/OutboundConversation.hpp"
#include "base/src/cpp/utils/Uri.hpp"
#include "mt-search/comms/src/cpp/OefSearchEndpoint.hpp"
#include "base/src/cpp/monitoring/Counter.hpp"


template <typename IN_PROTO, typename OUT_PROTO>
class DapConversationTask
    : public StateMachineTask<DapConversationTask<IN_PROTO, OUT_PROTO>>
    , std::enable_shared_from_this<DapConversationTask<IN_PROTO, OUT_PROTO>>
    , Waitable
{
public:

  using StateResult = typename StateMachineTask<DapConversationTask>::Result;
  using EntryPoint  = typename StateMachineTask<DapConversationTask>::EntryPoint;
  using MessageHandler = std::function<void(std::shared_ptr<OUT_PROTO>)>;
  using ErrorHandler   = std::function<void(const std::string&, const std::string&)>;

  static constexpr char const *LOGGING_NAME = "DapConversationTask";

  DapConversationTask(
      std::string dap_name,
      std::string path,
      uint32_t msg_id,
      std::shared_ptr<IN_PROTO> initiator,
      std::shared_ptr<OutboundConversations> outbounds,
      std::shared_ptr<OefSearchEndpoint> endpoint
  )
      :  StateMachineTask<DapConversationTask>(this,
                                               {&DapConversationTask::createConv,
                                                &DapConversationTask::handleResponse
                                               }
                                               )
      , initiator(std::move(initiator))
      , outbounds(std::move(outbounds))
      , endpoint(std::move(endpoint))
      , dap_name_{std::move(dap_name)}
      , path_{std::move(path)}
      , msg_id_(msg_id)
  {
    FETCH_LOG_INFO(LOGGING_NAME, "Task created.");
    task_created = std::make_shared<Counter>("mt-search.dap."+dap_name_+"."+path_+".created");
    task_errored = std::make_shared<Counter>("mt-search.dap."+dap_name_+"."+path_+".errored");
    task_succeeded = std::make_shared<Counter>("mt-search.dap."+dap_name_+"."+path_+".succeeded");

    (*task_created)++;
  }

  virtual ~DapConversationTask()
  {
    FETCH_LOG_INFO(LOGGING_NAME, "Task gone.");
  }

  StateResult createConv(void)
  {
    auto this_sp = this->shared_from_this();
    std::weak_ptr<Task> this_wp = this_sp;
    FETCH_LOG_INFO(LOGGING_NAME, "Start: DapName: ", dap_name_);
    FETCH_LOG_INFO(LOGGING_NAME, "***PATH: ", path_);
    conversation = outbounds -> startConversation(
        Uri("outbound://"+dap_name_+"/"+path_),
        initiator
    );

    if (conversation -> makeNotification().Then( [this_wp](){ auto sp = this_wp.lock(); if (sp) { sp -> makeRunnable(); } } ).Waiting())
    {
      FETCH_LOG_INFO(LOGGING_NAME, "Sleeping");
      return DapConversationTask::StateResult(1, DEFER);
    }
    FETCH_LOG_INFO(LOGGING_NAME, "NOT Sleeping");
    return DapConversationTask::StateResult(1, COMPLETE);
  }

  virtual StateResult handleResponse(void)
  {
    FETCH_LOG_INFO(LOGGING_NAME, "Woken ");
    FETCH_LOG_INFO(LOGGING_NAME, "Response.. ",
                   conversation -> getAvailableReplyCount()
    );

    if (conversation -> getAvailableReplyCount() == 0){
      (*task_errored)++;
      if (errorHandler)
      {
        errorHandler(dap_name_, path_);
      }
      wake();
      return DapConversationTask::StateResult(0, ERRORED);
    }

    auto resp = conversation->getReply(0);
    if (!resp){
      FETCH_LOG_ERROR(LOGGING_NAME, "Got nullptr as reply");
      (*task_errored)++;
      if (errorHandler)
      {
        errorHandler(dap_name_, path_);
      }
      wake();
      return DapConversationTask::StateResult(0, ERRORED);
    }
    auto response = std::static_pointer_cast<OUT_PROTO>(resp);
    if (messageHandler)
    {
      messageHandler(std::move(response));
      (*task_succeeded)++;
      wake();
    }
    else
    {
      (*task_errored)++;
      if (errorHandler)
      {
        errorHandler(dap_name_, path_);
      }
      wake();
      return DapConversationTask::StateResult(0, ERRORED);
    }

    FETCH_LOG_INFO(LOGGING_NAME, "COMPLETE");

    return DapConversationTask::StateResult(0, COMPLETE);
  }

  MessageHandler messageHandler;
  ErrorHandler errorHandler;

protected:
  std::shared_ptr<IN_PROTO> initiator;
  std::shared_ptr<OutboundConversations> outbounds;
  std::shared_ptr<OutboundConversation> conversation;
  std::shared_ptr<OefSearchEndpoint> endpoint;
  std::string dap_name_;
  std::string path_;
  uint32_t msg_id_;

  std::shared_ptr<Counter> task_created;
  std::shared_ptr<Counter> task_errored;
  std::shared_ptr<Counter> task_succeeded;

private:
  DapConversationTask(const DapConversationTask &other) = delete; // { copy(other); }
  DapConversationTask &operator=(const DapConversationTask &other) = delete; // { copy(other); return *this; }
  bool operator==(const DapConversationTask &other) = delete; // const { return compare(other)==0; }
  bool operator<(const DapConversationTask &other) = delete; // const { return compare(other)==-1; }
};
