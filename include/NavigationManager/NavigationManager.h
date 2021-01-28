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
#include <thread>
#include <sstream>
#include "httplib.h"


class HttpServer;
struct NavigationMapConfig {
  std::string map_path;
  double x_scale;
  double y_scale;
  double globalPositionOfTopLeft_x;
  double globalPositionOfTopLeft_y;  
  int32_t image_columns;
  int32_t image_rows;
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
    bool refreshMap();
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
};


inline std::string toJson(const RTC::RangeData& range, const long step) {
  std::cout << "ToJson data Range Data conversion...." << std::endl;
  std::stringstream ss;
  ss << "{\"base2laser\":[" << range.geometry.geometry.pose.position.x << "," << range.geometry.geometry.pose.position.y << "],"
    //ss << "{\"base2laser\":[" << 0.0 << ","
    //     << 0.0 << "],"    
     <<"\"amin\":" << range.config.minAngle << ","
     <<"\"amax\":" << range.config.maxAngle << ","
     <<"\"ares\":" << range.config.angularRes << ","
     <<"\"minRange\":" << range.config.minRange << ","
     <<"\"maxRange\":" << range.config.maxRange << ","
     <<"\"step\":" << step << ","
     <<"\"ranges\":[";
  const int len = range.ranges.length()/4;
  for(int i = 0;i < len;i++) {
    ss << range.ranges[i*step];
    if (i != len-1) ss << ",";
  }
  ss << "]}";
  return ss.str();
}

inline std::string toJson(const NavigationMapConfig& config) {
  std::stringstream ss;
  ss << "{\"map_path\":\"" << config.map_path << "\","
     << "\"x_scale\":" << config.x_scale << ","
     << "\"y_scale\":" << config.y_scale << ","
     << "\"globalPositionOfTopLeft_x\":" << config.globalPositionOfTopLeft_x << ","
     << "\"globalPositionOfTopLeft_y\":" << config.globalPositionOfTopLeft_y << ","
     << "\"image_columns\":" << config.image_columns << ","
     << "\"image_rows\":" << config.image_rows << "}";
  return ss.str();
    
}

inline std::string toJson(const RTC::TimedPose2D& pose) {
  std::stringstream ss;
  ss << "{\"tm\":{\"sec\":" << pose.tm.sec << ",\"nsec\":" << pose.tm.nsec << "},"
     << "\"data\":{\"position\":{\"x\":" << pose.data.position.x << ",\"y\":" << pose.data.position.y << "},\"heading\":" << pose.data.heading << "}}";
  return ss.str();
}

class HttpServer {
private:
  httplib::Server server_;
  NavigationManager* m_pRTC;
public:
  void setRTC(NavigationManager* pRTC) { m_pRTC = pRTC; } 
public:
  HttpServer() {}

  void runBackground(const std::string& baseDir, const std::string& address, const int port, const double timeoutSec) {
    server_.set_mount_point(nullptr, baseDir.c_str());
    
    thread_ = std::make_unique<std::thread>([address, port, this]() {
      server_.listen(address.c_str(), port);
    });
  }
  ~HttpServer() {
    terminateBackground();
    if (thread_) {
      thread_->join();
    }
  }

  void terminateBackground() {
    server_.stop();
  }

  void initServer() {
    std::string endpoint = "/api/";
    server_.Get((endpoint + "currentRobotPose").c_str(), [this](const httplib::Request& req, httplib::Response& res) {
      res.status = 200;
      res.version = "1.0";
      res.body = toJson(m_pRTC->m_currentRobotPose);
    });

    server_.Get((endpoint + "rangeScan").c_str(), [this](const httplib::Request& req, httplib::Response& res) {
      res.status = 200;
      res.version = "1.0";
      res.body = toJson(m_pRTC->m_range, 4);
    });

    server_.Put((endpoint + "map/refresh").c_str(), [this](const httplib::Request& req, httplib::Response& res) {
      if (m_pRTC->refreshMap()) {
	res.status = 200;
	res.version = "1.0";
	res.body = toJson(m_pRTC->m_mapConfig);
      } else {
	res.status = 402;
	res.version = "1.0";
	res.body = "Failed to load map from MapServer";
      }
    });

    server_.Put((endpoint + "key").c_str(), [this](const httplib::Request& req, httplib::Response& res) {
      m_pRTC->keyEvent(req.body);
      res.status = 200;
      res.version = "1.0";
    });

    server_.Get((endpoint + "map/config").c_str(), [this](const httplib::Request& req, httplib::Response& res) {
      res.status = 200;
      res.version = "1.0";
      res.body = toJson(m_pRTC->m_mapConfig);
    });
  }
  
  /*
  void response(const std::string& path, const std::string& method, const std::string& contentType, std::function<nerikiri::Response(const nerikiri::Request&)> callback) {
    if (method == "GET") {
      server_.Get(path.c_str(), [callback, contentType](const httplib::Request& req, httplib::Response& res) {
	apply(res, callback(convert(req)));
      });
      
    } else if (method == "PUT") {
      server_.Put(path.c_str(), [callback, contentType](const httplib::Request& req, httplib::Response& res) {
	apply(res, callback(convert(req)));
      });
    }
    else if (method == "POST") {
      server_.Post(path.c_str(), [callback, contentType](const httplib::Request& req, httplib::Response& res) {
	apply(res, callback(convert(req)));
      });
    }
    else if (method == "DELETE") {
      server_.Delete(path.c_str(), [callback, contentType](const httplib::Request& req, httplib::Response& res) {
	apply(res, callback(convert(req)));
      });
    }
  }

  inline nerikiri::Request convert(const httplib::Request &req) {
    std::vector<nerikiri::Header> ret;
    for(const auto& [k, v] : req.headers) {
      ret.push_back(Header(k, v));
    }
    auto r = nerikiri::Request(req.method, req.body, ret, req.matches);
    for(auto p : req.params) {
      r.params[p.first] = p.second;
    }
    return r;
  }

  inline void apply(httplib::Response &response, nerikiri::Response &&r) {
    response.status = r.status;
    response.version = r.version;
    if(r.is_file_) {
      auto size = r.file_.tellg();

      response.body.resize(static_cast<size_t>(size));
      r.file_.read(&response.body[0], size);
    } else {
      response.set_content(r.body, r.contentType.c_str());
    }
  }

  */
  
    
  private:
    std::unique_ptr<std::thread> thread_;

};


extern "C"
{
  DLL_EXPORT void NavigationManagerInit(RTC::Manager* manager);
};

#endif // NAVIGATIONMANAGER_H
