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

#include "logging/logging.hpp"
#include "oef-base/comms/IOefTaskFactory.hpp"
#include "oef-core/agents/Agents.hpp"

#include "agent.pb.h"

namespace google {
namespace protobuf {
class Message;
};
};  // namespace google

class OefFunctionsTaskFactory : public IOefTaskFactory<OefAgentEndpoint>
{
public:
  static constexpr char const *LOGGING_NAME = "OefFunctionsTaskFactory";

  OefFunctionsTaskFactory(const std::string &core_key, std::shared_ptr<Agents> agents,
                          std::string                            agent_public_key,
                          std::shared_ptr<OutboundConversations> outbounds)
    : IOefTaskFactory(outbounds)
    , agents_{std::move(agents)}
    , agent_public_key_{std::move(agent_public_key)}
    , core_key_{core_key}
  {}
  virtual ~OefFunctionsTaskFactory()
  {}

  virtual void processMessage(ConstCharArrayBuffer &data);
  // Process the message, throw exceptions if they're bad.

  virtual void endpointClosed(void);

protected:
private:
  OefFunctionsTaskFactory(const OefFunctionsTaskFactory &other) = delete;
  OefFunctionsTaskFactory &operator=(const OefFunctionsTaskFactory &other)  = delete;
  bool                     operator==(const OefFunctionsTaskFactory &other) = delete;
  bool                     operator<(const OefFunctionsTaskFactory &other)  = delete;

private:
  std::shared_ptr<Agents> agents_;
  std::string             agent_public_key_;
  std::string             core_key_;
};