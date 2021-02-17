// -*- C++ -*-
/*!
 * @file  NavigationManager.cpp
 * @brief Navigation Manager Component On the WEB
 * @date $Date$
 *
 * $Id$
 */

#include "NavigationManager.h"

inline bool checkServicePortHasConnection(const char* instanceName, RTC::CorbaPort& servicePort) {                                                                                                                                                                           
  RTC::ConnectorProfileList_var conProfList(servicePort.get_connector_profiles());
  if (conProfList->length() == 0) {
          //RTC_ERROR(("NavigationManager::refreshPF() called. But no connection."));
          return false;
  }
  auto& conProf = conProfList[0];
  for (int i = 0; i < 2; i++) {
          RTC::PortProfile_var pprof(conProfList[0].ports[i]->get_port_profile());
          RTC::ComponentProfile_var cprof(pprof->owner->get_component_profile());
          if (instanceName != cprof->instance_name) {
                  RTC::ExecutionContextList_var ecList(pprof->owner->get_owned_contexts());
                  if (ecList->length() == 0) {
                        return false;
                  }
                  RTC::LifeCycleState state = ecList[0]->get_component_state(pprof->owner);
                  if (state != RTC::LifeCycleState::ACTIVE_STATE) {
                          //RTC_WARN(("NavigationManager::refreshPF() failed. Connected Component is not activated."));
                          std::cout << (("NavigationManager::refreshPF() failed. Connected Component is not activated.")) << std::endl;;
                          return false;
                  }
                  break;
          }
  }

  return true;
}

#include <thread>
#include <memory>
#include <opencv2/opencv.hpp>
// Module specification
// <rtc-template block="module_spec">
static const char* navigationmanager_spec[] =
  {
    "implementation_id", "NavigationManager",
    "type_name",         "NavigationManager",
    "description",       "Navigation Manager Component On the WEB",
    "version",           "1.1.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Navigation",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",

    "conf.default.base_dir", "../",
    "conf.default.address", "0.0.0.0",
    "conf.default.port", "8088",
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
    m_mclServicePort("mclService"),
    m_pathPlannerPort("pathPlanner"),
    m_mapperPort("mapper"),
    m_pServer(nullptr),
    m_mclInfo(new NAVIGATION::MCLInfo())
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
NavigationManager::~NavigationManager()
{
}


bool NavigationManager::refreshPF() {
  std::cout << "[NavigationManager] NavigationManager::refreshPF() called." << std::endl;
  std::lock_guard<std::mutex> g(mcl_mutex_);
  if( !checkServicePortHasConnection (getInstanceName(), m_mclServicePort)) {
    return false;
  }
  auto ret = m_NAVIGATION_MonteCarloLocalization->requestParticles(m_mclInfo);
  if (ret != NAVIGATION::MCL_RETVAL_OK) {
    std::cout << "[NavigationManager] failed to get pf" << std::endl;
    return false;    
  }
  //  m_mclInfo = *info;
  std::cout << "[NavigationManager] NavigationManager::refreshPF() exit." << std::endl;  
  return true;
}

bool NavigationManager::refreshMap(const MapParam& req_param) {
  std::cout << "[NavigationManager] NavigationManager::refreshMap() called." << std::endl;  
  NAVIGATION::OccupancyGridMap_var map;
  NAVIGATION::OccupancyGridMapRequestParam_var param(new NAVIGATION::OccupancyGridMapRequestParam());;
  param->globalPositionOfCenter.position.x = req_param.globalPositionOfCenter.x;
  param->globalPositionOfCenter.position.y = req_param.globalPositionOfCenter.y;
  param->globalPositionOfCenter.heading = req_param.globalPositionOfCenter.a;
  param->sizeOfMap.height = req_param.sizeOfMap.h; // Negative Value ... Maximum Size.
  param->sizeOfMap.width = req_param.sizeOfMap.w; // Negative Value ... Maximum Size.
  param->sizeOfGrid.height = req_param.sizeOfGrid.h;
  param->sizeOfGrid.width = req_param.sizeOfGrid.w;
  if (!checkServicePortHasConnection(getInstanceName(), m_mapServerPort)) {
    return false;
  }
  auto ret = m_NAVIGATION_OccupancyGridMapServer->requestLocalMap(param, map);
  if (ret != NAVIGATION::MAP_OK) {
      std::cout << "[NavigationManager] failed to get map" << std::endl;
      return false;
  }

  int col = map->config.sizeOfGridMap.width; // map->config.sizeOfGrid.width;
  int row = map->config.sizeOfGridMap.height; // map->config.sizeOfGrid.height;
  int typ = CV_8UC1; // grayscale
  cv::Mat img(row, col, typ);
  std::cout << "img(" << row << " x " << col << ")" << std::endl;
  for(int r = 0;r < row;r++) {
    for(int c = 0;c < col;c++) {
      //img.at<uchar>(r, c, 0) = map->cells[r*col+c];
      img.data[r*col + c] = map->cells[r*col + c];
    }
  }
  cv::Mat tmp;
  cv::cvtColor(img, tmp, cv::COLOR_GRAY2BGR);
  std::string map_filename = "/map_temp.png";
  auto path = m_base_dir + map_filename;
  
  cv::imwrite(path, tmp);
  std::cout << "map is saved to " << path << std::endl;
  m_mapConfig.map_path = map_filename;
  m_mapConfig.x_scale = map->config.sizeOfGrid.width;
  m_mapConfig.y_scale = map->config.sizeOfGrid.height;
  m_mapConfig.globalPositionOfTopLeft_x = map->config.globalPositionOfTopLeft.position.x;
  m_mapConfig.globalPositionOfTopLeft_y = map->config.globalPositionOfTopLeft.position.y;
  m_mapConfig.image_columns = img.cols;
  m_mapConfig.image_rows    = img.rows;


  return true;
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
  m_pathPlannerPort.registerConsumer("NAVIGATION_PathPlanner", "NAVIGATION::PathPlanner", m_NAVIGATION_PathPlanner);
  m_mapperPort.registerConsumer("NAVIGATION_OccupancyGridMapper", "NAVIGATION::OccupancyGridMapper", m_NAVIGATION_OccupancyGridMapper);

  // Set CORBA Service Ports
  addPort(m_mapServerPort);
  addPort(m_mclServicePort);
  addPort(m_pathPlannerPort);
  addPort(m_mapperPort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  bindParameter("base_dir", m_base_dir, "../");
  bindParameter("address", m_address, "0.0.0.0");
  bindParameter("port", m_port, "8080");
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
  std::cout << "[NavigationManager] onActivated called" << std::endl;
  m_pServer = createHttpServer();
  m_pServer->setRTC(this);
  m_pServer->initServer();
  m_pServer->runBackground(m_base_dir, m_address, m_port, 10.0);

  m_currentRobotPose.tm.sec = 0;
  m_currentRobotPose.tm.nsec = 0;
  m_currentRobotPose.data.position.x = 0;
  m_currentRobotPose.data.position.y = 0;
  m_currentRobotPose.data.heading = 0;


  m_targetVelocity.data.vx = m_targetVelocity.data.vy = m_targetVelocity.data.va = 0;
  std::cout << "[NavigationManager] onActivated exit" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t NavigationManager::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "[NavigationManager] onDeactivated called" << std::endl;
  m_pServer->terminateBackground();
  delete m_pServer;
  m_pServer = nullptr;
  std::cout << "[NavigationManager] onDeactivated exit" << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t NavigationManager::onExecute(RTC::UniqueId ec_id)
{
  if (m_currentRobotPoseIn.isNew()) {
    m_currentRobotPoseIn.read();
    std::cout << "currentPose:" << m_currentRobotPose.data.position.x << ", " << m_currentRobotPose.data.position.y  << ", " <<  m_currentRobotPose.data.heading << std::endl;
  }

  if (m_rangeIn.isNew()) {
    m_rangeIn.read();
    //range_ = m_range;
  }

  setTimestamp(m_targetVelocity);
  m_targetVelocityOut.write();
  
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


