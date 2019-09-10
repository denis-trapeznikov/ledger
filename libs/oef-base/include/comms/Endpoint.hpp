#pragma once

#include "base/src/cpp/comms/EndpointBase.hpp"
#include "base/src/cpp/comms/RingBuffer.hpp"
#include "base/src/cpp/comms/ISocketOwner.hpp"
#include <boost/asio.hpp>

#include "base/src/cpp/comms/Core.hpp"
#include "base/src/cpp/comms/IMessageReader.hpp"
#include "base/src/cpp/comms/IMessageWriter.hpp"
#include "fetch_teams/ledger/logger.hpp"
#include <iostream>


template <typename TXType>
class Endpoint
  : public EndpointBase<TXType>
  , public std::enable_shared_from_this<Endpoint<TXType>>
{
public:
  using EndpointBase<TXType>::state;
  using EndpointBase<TXType>::readBuffer;
  using EndpointBase<TXType>::sendBuffer;
  using typename EndpointBase<TXType>::message_type;

  using ConfigMap = typename EndpointBase<TXType>::ConfigMap;
  using Socket    = typename EndpointBase<TXType>::Socket;

  static constexpr char const *LOGGING_NAME = "Endpoint";

  Endpoint(const Endpoint &other) = delete;
  Endpoint &operator=(const Endpoint &other) = delete;
  bool operator==(const Endpoint &other) = delete;
  bool operator<(const Endpoint &other) = delete;

  Endpoint(
      Core &core
      ,std::size_t sendBufferSize
      ,std::size_t readBufferSize
      ,ConfigMap configMap
  );

  virtual ~Endpoint();

  virtual Socket& socket() override
  {
    return sock;
  }

protected:
  virtual void async_read(const std::size_t& bytes_needed) override;
  virtual void async_write() override;
  virtual bool is_eof(const boost::system::error_code& ec) const override;

protected:
  Socket sock;
};
