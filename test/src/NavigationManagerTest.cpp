// -*- C++ -*-
/*!
 * @file  NavigationManagerTest.cpp
 * @brief Navigation Manager Component On the WEB
 * @date $Date$
 *
 * $Id$
 */

#include "NavigationManagerTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* navigationmanager_spec[] =
  {
    "implementation_id", "NavigationManagerTest",
    "type_name",         "NavigationManagerTest",
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
NavigationManagerTest::NavigationManagerTest(RTC::Manager* manager)
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
NavigationManagerTest::~NavigationManagerTest()
{
}



RTC::ReturnCode_t NavigationManagerTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("targetVelocity", m_targetVelocityIn);

  // Set OutPort buffer
  addOutPort("currentRobotPose", m_currentRobotPoseOut);
  addOutPort("range", m_rangeOut);

  // Set service provider to Ports
  m_mapServerPort.registerProvider("NAVIGATION_OccupancyGridMapServer", "NAVIGATION::OccupancyGridMapServer", m_NAVIGATION_OccupancyGridMapServer);
  m_mclServicePort.registerProvider("NAVIGATION_MonteCarloLocalization", "NAVIGATION::MonteCarloLocalization", m_NAVIGATION_MonteCarloLocalization);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_mapServerPort);
  addPort(m_mclServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t NavigationManagerTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManagerTest::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManagerTest::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t NavigationManagerTest::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t NavigationManagerTest::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t NavigationManagerTest::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t NavigationManagerTest::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManagerTest::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManagerTest::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManagerTest::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NavigationManagerTest::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void NavigationManagerTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(navigationmanager_spec);
    manager->registerFactory(profile,
                             RTC::Create<NavigationManagerTest>,
                             RTC::Delete<NavigationManagerTest>);
  }

};


