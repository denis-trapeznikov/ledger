#pragma once

#include <vector>
#include <memory>

#include <boost/asio.hpp>

#include "base/src/cpp/comms/ConstCharArrayBuffer.hpp"

class OutboundConversations;

template <typename OefEndpoint>
class IOefTaskFactory
{
  friend OefEndpoint;
public:
  using buffer = boost::asio::const_buffer;
  using buffers = std::vector<buffer>;

  IOefTaskFactory(std::shared_ptr<OefEndpoint> the_endpoint, std::shared_ptr<OutboundConversations> outbounds)
    : outbounds(outbounds)
    , endpoint(the_endpoint)
  {
  }
  IOefTaskFactory(std::shared_ptr<OutboundConversations> outbounds)
    : outbounds(outbounds)
  {
  }
  virtual ~IOefTaskFactory()
  {
  }

  virtual void processMessage(ConstCharArrayBuffer &data) = 0;
  // Process the message, throw exceptions if they're bad.
protected:
   std::shared_ptr<OutboundConversations> outbounds;

  template<class PROTO>
  void read(PROTO &proto, ConstCharArrayBuffer &chars, std::size_t expected_size)
  {
    std::istream is(&chars);
    auto current = chars.remainingData();
    auto result = proto . ParseFromIstream(&is);
    auto eaten = current - chars.remainingData();
    if (!result)
    {
      throw std::invalid_argument("Failed proto deserialisation.");
    }
    if (eaten != expected_size)
    {
      throw std::invalid_argument(
                                  std::string("Proto deserialisation used ")
                                  + std::to_string(eaten)
                                  + " bytes instead of "
                                  + std::to_string(expected_size)
                                  + ".");
    }
  }

  template<class PROTO>
  void read(PROTO &proto, const std::string &s)
  {
    auto result = proto . ParseFromString(s);
    if (!result)
    {
      throw std::invalid_argument("Failed proto deserialisation.");
    }
  }

  template<class PROTO>
  void read(PROTO &proto, ConstCharArrayBuffer &chars)
  {
    std::istream is(&chars);
    auto result = proto . ParseFromIstream(&is);
    if (!result)
    {
      throw std::invalid_argument("Failed proto deserialisation.");
    }
    if (chars.remainingData() != 0)
    {
      throw std::invalid_argument(
                                  std::string("Proto deserialisation left used ")
                                  + std::to_string(chars.remainingData())
                                  + "unused bytes.");
    }
  }

  void successor(std::shared_ptr<IOefTaskFactory> factory)
  {
    endpoint -> setFactory(factory);
  }

  std::shared_ptr<OefEndpoint> getEndpoint() { return endpoint; }
  virtual void endpointClosed(void) = 0;
private:
  std::shared_ptr<OefEndpoint> endpoint;

  IOefTaskFactory(const IOefTaskFactory &other) = delete;
  IOefTaskFactory &operator=(const IOefTaskFactory &other) = delete;
  bool operator==(const IOefTaskFactory &other) = delete;
  bool operator<(const IOefTaskFactory &other) = delete;

};
