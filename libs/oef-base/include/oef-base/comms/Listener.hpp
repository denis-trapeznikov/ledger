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

#include "network/fetch_asio.hpp"
#include "oef-base/comms/ISocketOwner.hpp"

#include <memory>

using asio::ip::tcp;

class Core;

class Listener
{
public:
  using CONN_CREATOR = std::function<std::shared_ptr<ISocketOwner>(Core &core)>;

  Listener(Core &core, unsigned short int port);
  virtual ~Listener();

  void start_accept();
  void handle_accept(std::shared_ptr<ISocketOwner> new_connection, std::error_code const &error);

  std::shared_ptr<tcp::acceptor> acceptor;
  CONN_CREATOR                   creator;

private:
  Core &core;
};