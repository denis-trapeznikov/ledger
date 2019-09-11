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
#include "oef-base/proto_comms/ProtoMessageEndpoint.hpp"
#include <memory>

class Core;
class SearchTaskFactory;
class ProtoPathMessageReader;
class ProtoPathMessageSender;

class OefSearchEndpoint
  : public EndpointPipe<
        ProtoMessageEndpoint<std::pair<Uri, std::shared_ptr<google::protobuf::Message>>,
                             ProtoPathMessageReader, ProtoPathMessageSender>>,
    public std::enable_shared_from_this<OefSearchEndpoint>
{
public:
  static constexpr char const *LOGGING_NAME = "OefSearchEndpoint";

  using TXType = std::pair<Uri, std::shared_ptr<google::protobuf::Message>>;
  using ProtoEndpoint =
      ProtoMessageEndpoint<TXType, ProtoPathMessageReader, ProtoPathMessageSender>;
  using Parent = EndpointPipe<ProtoEndpoint>;
  using SELF_P = std::shared_ptr<OefSearchEndpoint>;
  using Parent::endpoint;

  explicit OefSearchEndpoint(std::shared_ptr<ProtoEndpoint> endpoint);
  virtual ~OefSearchEndpoint();

  void setFactory(std::shared_ptr<SearchTaskFactory> new_factory);
  void setup();

  virtual void go(void) override
  {
    FETCH_LOG_INFO(LOGGING_NAME, "------------------> OefSearchEndpoint::go");

    SELF_P self = shared_from_this();
    while (!go_functions.empty())
    {
      go_functions.front()(self);
      go_functions.pop_front();
    }

    Parent::go();
  }

  void close(const std::string &reason);
  void setState(const std::string &stateName, bool value);
  bool getState(const std::string &stateName) const;

  std::size_t getIdent() const
  {
    return ident;
  }

  void addGoFunction(std::function<void(SELF_P)> func)
  {
    go_functions.push_back(func);
  }

protected:
private:
  std::map<std::string, bool>            states;
  std::size_t                            ident;
  std::shared_ptr<SearchTaskFactory>     factory;
  std::list<std::function<void(SELF_P)>> go_functions;

  OefSearchEndpoint(const OefSearchEndpoint &other) = delete;
  OefSearchEndpoint &operator=(const OefSearchEndpoint &other)  = delete;
  bool               operator==(const OefSearchEndpoint &other) = delete;
  bool               operator<(const OefSearchEndpoint &other)  = delete;
};