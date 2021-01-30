// -*- C++ -*-
/*!
 * @file  NavigationManager.h
 * @brief Navigation Manager Component On the WEB
 * @date  $Date$
 *
 * $Id$
 */

#ifndef NAVIGATIONMANAGER_H
#define NAVIGATIONMANAGER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "MobileRobotNavigationStub.h"
#include "ExtendedDataTypesStub.h"
#include "InterfaceDataTypesStub.h"

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>


#include <memory>
#include <mutex>
#include <thread>
#include <sstream>


#include "HttpServer.h"

struct Pose {
  double x, y, a;
};

struct Size {
  double w,h;
};

struct MapParam {
  Pose globalPositionOfCenter;
  Size sizeOfMap;
  Size sizeOfGrid;
};
  
 

/*!
 * @class NavigationManager
 * @brief Navigation Manager Component On the WEB
 *
 */
class NavigationManager
  : public RTC::DataFlowComponentBase
{
 public:
  friend class HttpServer;
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  NavigationManager(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~NavigationManager();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  std::string m_base_dir;
  std::string m_address;
  int m_port;

  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedPose2D m_currentRobotPose;
  /*!
   */
  RTC::InPort<RTC::TimedPose2D> m_currentRobotPoseIn;
  RTC::RangeData m_range;
  /*!
   */
  RTC::InPort<RTC::RangeData> m_rangeIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedVelocity2D m_targetVelocity;
  /*!
   */
  RTC::OutPort<RTC::TimedVelocity2D> m_targetVelocityOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_mapServerPort;
  /*!
   */
  RTC::CorbaPort m_mclServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  /*!
   */
  RTC::CorbaConsumer<NAVIGATION::OccupancyGridMapServer> m_NAVIGATION_OccupancyGridMapServer;
  /*!
   */
  RTC::CorbaConsumer<NAVIGATION::MonteCarloLocalization> m_NAVIGATION_MonteCarloLocalization;
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>
  HttpServer* m_pServer;

public:
  bool refreshMap(const MapParam& param);

  std::mutex mcl_mutex_;
  NAVIGATION::MCLInfo_var m_mclInfo;
  bool refreshPF();
  
  NavigationMapConfig m_mapConfig;

  void keyEvent(const std::string& val) {
    const double vx = 1.0;
    const double va = 1.0;
    if (val == "1") {
      m_targetVelocity.data.vx = vx;
      m_targetVelocity.data.vy = 0;
      m_targetVelocity.data.va = 0;
    } else if (val == "2") {
      m_targetVelocity.data.vx = 0;
      m_targetVelocity.data.vy = 0;
      m_targetVelocity.data.va = -va;

    } else if (val == "3") {
      m_targetVelocity.data.vx = -vx;
      m_targetVelocity.data.vy = 0;
      m_targetVelocity.data.va = 0;
    } else if (val == "4") {
      m_targetVelocity.data.vx = 0;
      m_targetVelocity.data.vy = 0;
      m_targetVelocity.data.va = va;
    } else if (val == "0") {
      m_targetVelocity.data.vx = 0;
      m_targetVelocity.data.vy = 0;
      m_targetVelocity.data.va = 0;

    } else {
      std::cout << "Unknown key event'" << val << "'" << std::endl;
    }
  }

  const RTC::TimedPose2D& getCurrentRobotPose() { return m_currentRobotPose; }
  const RTC::RangeData& getRange() { return m_range; }
  const NavigationMapConfig& getMapConfig() { return m_mapConfig; }
  const NAVIGATION::MCLInfo& getMCLInfo() { return m_mclInfo; }
};



extern "C"
{
  DLL_EXPORT void NavigationManagerInit(RTC::Manager* manager);
};

#endif // NAVIGATIONMANAGER_H
