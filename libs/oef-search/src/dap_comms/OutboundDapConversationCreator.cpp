#include "OutboundDapConversationCreator.hpp"

#include "base/src/cpp/proto_comms/ProtoMessageEndpoint.hpp"
#include "base/src/cpp/threading/StateMachineTask.hpp"
#include "base/src/cpp/utils/Uri.hpp"
#include "fetch_teams/ledger/logger.hpp"

#include "base/src/cpp/conversation/OutboundConversationWorkerTask.hpp"

#include <google/protobuf/message.h>


OutboundDapConversationCreator::OutboundDapConversationCreator(size_t thread_group_id, const Uri &dap_uri, Core &core, std::shared_ptr<OutboundConversations> outbounds)
{
  worker = std::make_shared<OutboundConversationWorkerTask>(core, dap_uri, outbounds, ident2conversation);

  worker -> setThreadGroupId(thread_group_id);

  worker -> submit();
}

OutboundDapConversationCreator::~OutboundDapConversationCreator()
{
  worker.reset();
}

std::shared_ptr<OutboundConversation> OutboundDapConversationCreator::start(const Uri &target_path, std::shared_ptr<google::protobuf::Message> initiator)
{
  FETCH_LOG_INFO(LOGGING_NAME, "Starting search conversation...");
  auto this_id = ident++;

  std::shared_ptr<OutboundConversation> conv;

  /*if (target_path.path == "/update")
  {
    conv = std::make_shared<OutboundTypedConversation<fetch::oef::pb::UpdateResponse>>(this_id, target_path, initiator);
  }
  else if (target_path.path == "/remove")
  {
    conv = std::make_shared<OutboundTypedConversation<fetch::oef::pb::RemoveResponse>>(this_id, target_path, initiator);
  }
  else if (target_path.path == "/search")
  {
    conv = std::make_shared<OutboundTypedConversation<fetch::oef::pb::SearchResponse>>(this_id, target_path, initiator);
  }
  else
  {
    throw std::invalid_argument(target_path.path + " is not a valid target, to start a OutboundDapConversationCreator!");
  }*/

  ident2conversation[this_id] = conv;
  worker -> post(conv);
  return conv;
}
