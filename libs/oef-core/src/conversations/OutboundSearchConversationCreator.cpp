#include "oef-core/conversations/OutboundSearchConversationCreator.hpp"

#include "logging/logging.hpp"
#include "oef-base/proto_comms/ProtoMessageEndpoint.hpp"
#include "oef-base/threading/StateMachineTask.hpp"
#include "oef-base/utils/Uri.hpp"

#include "oef-core/conversations/SearchAddressUpdateTask.hpp"

#include "oef-base/conversation/OutboundConversationWorkerTask.hpp"
#include "oef-messages/dap_interface.hpp"
#include "oef-messages/search_message.hpp"
#include "oef-messages/search_query.hpp"
#include "oef-messages/search_remove.hpp"
#include "oef-messages/search_transport.hpp"
#include "oef-messages/search_update.hpp"

#include <google/protobuf/message.h>

std::map<unsigned long, std::shared_ptr<OutboundConversation>> ident2conversation;

// ------------------------------------------------------------------------------------------

class OutboundSearchConversationWorkerTask : public OutboundConversationWorkerTask
{
public:
  using ConversationMap = OutboundConversationWorkerTask::ConversationMap;

  static constexpr char const *LOGGING_NAME = "OutboundSearchConversationWorkerTask";

  OutboundSearchConversationWorkerTask(Core &core, const std::string &core_key, const Uri &core_uri,
                                       const Uri &                            search_uri,
                                       std::shared_ptr<OutboundConversations> outbounds,
                                       const ConversationMap &                ConversationMap)
    : OutboundConversationWorkerTask(core, search_uri, ConversationMap)
    , outbounds_(std::move(outbounds))
    , core_uri(core_uri)
    , core_key(core_key)
  {}

  virtual ~OutboundSearchConversationWorkerTask()
  {}

  friend class OutboundSearchConversationCreator;

protected:
  std::shared_ptr<OutboundConversations> outbounds_;

  Uri         core_uri;
  std::string core_key;

  void register_address()
  {
    auto address = std::make_shared<Address>();
    address->set_ip(core_uri.host);
    address->set_port(core_uri.port);
    address->set_key(core_key);
    address->set_signature("Sign");

    auto convTask = std::make_shared<SearchAddressUpdateTask>(address, outbounds_, nullptr);
    convTask->submit();
  }

  virtual bool connect() override
  {
    if (OutboundConversationWorkerTask::connect())
    {
      register_address();
      return true;
    }
    return false;
  }
};

// ------------------------------------------------------------------------------------------

OutboundSearchConversationCreator::OutboundSearchConversationCreator(
    const std::string &core_key, const Uri &core_uri, const Uri &search_uri, Core &core,
    std::shared_ptr<OutboundConversations> outbounds)
{
  worker = std::make_shared<OutboundSearchConversationWorkerTask>(
      core, core_key, core_uri, search_uri, outbounds, ident2conversation);

  worker->SetThreadGroupId(0);

  worker->submit();

  worker->connect();
}

OutboundSearchConversationCreator::~OutboundSearchConversationCreator()
{
  worker.reset();
}

std::shared_ptr<OutboundConversation> OutboundSearchConversationCreator::start(
    const Uri &target_path, std::shared_ptr<google::protobuf::Message> initiator)
{
  FETCH_LOG_INFO(LOGGING_NAME, "Starting search conversation...");
  auto this_id = ident++;

  std::shared_ptr<OutboundConversation> conv;

  if (target_path.path == "/update")
  {
    conv = std::make_shared<OutboundTypedConversation<Successfulness>>(this_id, target_path,
                                                                       initiator);
  }
  else if (target_path.path == "/remove")
  {
    conv = std::make_shared<OutboundTypedConversation<Successfulness>>(this_id, target_path,
                                                                       initiator);
  }
  else if (target_path.path == "/search")
  {
    conv = std::make_shared<OutboundTypedConversation<IdentifierSequence>>(this_id, target_path,
                                                                           initiator);
  }
  else
  {
    throw std::invalid_argument(
        target_path.path + " is not a valid target, to start a OutboundSearchConversationCreator!");
  }

  ident2conversation[this_id] = conv;
  worker->post(conv);
  return conv;
}