// -*- C++ -*-
/*!
 * @file  NavigationManager.cpp
 * @brief Navigation Manager Component On the WEB
 * @date $Date$
 *
 * $Id$
 */

#include "NavigationManager.h"

// Module specification
// <rtc-template block="module_spec">
static const char* navigationmanager_spec[] =
  {
    "implementation_id", "NavigationManager",
    "type_name",         "NavigationManager",
    "description",       "Navigation Manager Component On the WEB",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Navigation",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
NavigationManager::NavigationManager(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_currentRobotPoseIn("currentRobotPose", m_currentRobotPose),
    m_rangeIn("range", m_range),
    m_targetVelocityOut("targetVelocity", m_targetVelocity),
    m_mapServerPort("mapServer"),
    m_mclServicePort("mclService")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
NavigationManager::~NavigationManager()
{
}



RTC::ReturnCode_t NavigationManager::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("currentRobotPose", m_currentRobotPoseIn);
  addInPort("range", m_rangeIn);

  // Set OutPort buffer
  addOutPort("targetVelocity", m_targetVelocityOut);

  // Set service provider to Ports

  // Set service consumers to Ports
  m_mapServerPort.registerConsumer("NAVIGATION_OccupancyGridMapServer", "NAVIGATION::OccupancyGridMapServer", m_NAVIGATION_OccupancyGridMapServer);
  m_mclServicePort.registerConsumer("NAVIGATION_MonteCarloLocalization", "NAVIGATION::MonteCarloLocalization", m_NAVIGATION_MonteCarloLocalization);

  // Set CORBA Service Ports
  addPort(m_mapServerPort);
  addPort(m_mclServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t NavigationManager::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManager::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManager::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t NavigationManager::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t NavigationManager::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t NavigationManager::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t NavigationManager::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManager::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManager::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManager::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManager::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void NavigationManagerInit(RTC::Manager* manager)
  {
    coil::Properties profile(navigationmanager_spec);
    manager->registerFactory(profile,
                             RTC::Create<NavigationManager>,
                             RTC::Delete<NavigationManager>);
  }

};


