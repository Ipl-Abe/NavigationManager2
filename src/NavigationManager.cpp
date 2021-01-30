// -*- C++ -*-
/*!
 * @file  NavigationManager.cpp
 * @brief Navigation Manager Component On the WEB
 * @date $Date$
 *
 * $Id$
 */

#include "NavigationManager.h"

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
    "version",           "1.0.0",
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
    m_pServer(nullptr)
  ,    m_mclInfo(new NAVIGATION::MCLInfo())
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
  //NAVIGATION::MCLInfo_var info(new NAVIGATION::MCLInfo());
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
  param->sizeOfMap.l = req_param.sizeOfMap.h; // Negative Value ... Maximum Size.
  param->sizeOfMap.w = req_param.sizeOfMap.w; // Negative Value ... Maximum Size.
  param->sizeOfGrid.l = req_param.sizeOfGrid.h;
  param->sizeOfGrid.w = req_param.sizeOfGrid.w;
  auto ret = m_NAVIGATION_OccupancyGridMapServer->requestLocalMap(param, map);
  if (ret != NAVIGATION::MAP_RETVAL_OK) {
      std::cout << "[NavigationManager] failed to get map" << std::endl;
      return false;
  }

  int col = map->config.sizeOfMap.w / map->config.sizeOfGrid.w;
  int row = map->config.sizeOfMap.l / map->config.sizeOfGrid.l;
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
  m_mapConfig.x_scale = map->config.sizeOfGrid.w;
  m_mapConfig.y_scale = map->config.sizeOfGrid.l;
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

  // Set CORBA Service Ports
  addPort(m_mapServerPort);
  addPort(m_mclServicePort);

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
  m_pServer = createHttpServer();
  m_pServer->setRTC(this);
  m_pServer->initServer();
  m_pServer->runBackground(m_base_dir, m_address, m_port, 10.0);

  m_currentRobotPose.tm.sec = 0;
  m_currentRobotPose.tm.nsec = 0;
  m_currentRobotPose.data.position.x = 0;
  m_currentRobotPose.data.position.y = 0;
  m_currentRobotPose.data.heading = 0;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t NavigationManager::onDeactivated(RTC::UniqueId ec_id)
{
  m_pServer->terminateBackground();
  delete m_pServer;
  m_pServer = nullptr;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t NavigationManager::onExecute(RTC::UniqueId ec_id)
{
  if (m_currentRobotPoseIn.isNew()) {
    m_currentRobotPoseIn.read();
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


