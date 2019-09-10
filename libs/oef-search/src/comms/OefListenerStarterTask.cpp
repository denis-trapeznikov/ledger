#include "OefListenerStarterTask.hpp"

#include <iostream>

#include "mt-search/comms/src/cpp/Oefv1Listener.hpp"
#include "base/src/cpp/comms/OefListenerSet.hpp"
#include "base/src/cpp/comms/Endpoint.hpp"


template <template <typename> class EndpointType>
ExitState OefListenerStarterTask<EndpointType>::run(void)
{
  // open port here.
  auto result = std::make_shared<Oefv1Listener<EndpointType>>(core, p, endpointConfig);

  result -> factoryCreator = initialFactoryCreator;

  result -> start();

  //when done add to the listeners
  listeners -> add(p, result);
  return ExitState::COMPLETE;
}

template class OefListenerStarterTask<Endpoint>;
