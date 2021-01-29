#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include "InterfaceDataTypes.hh"
#include "MobileRobotNavigation.hh"
class NavigationManager;

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

inline std::string toJson(NAVIGATION::MCLInfo info) {
  std::stringstream ss;
  ss << "{\"maxW\":" << info.maxWeight << ","
     << "\"maxWIndex\":" << info.maxWeightIndex << ","
     << "\"ps\":[";
  const int len = info.particles.length();
  for(int i = 0;i < len;i++) {
    const auto& p = info.particles[i];
    ss << "{\"p\":["
       << p.pose.position.x << "," << p.pose.position.y << ","
       << p.pose.heading << "],"
       <<  "\"w\":" << p.weight
       << "}";
    if (i != len-1) {
      ss << ",";
    }
  }
  ss << "]}";
  return ss.str();
}

inline std::string toJson(const RTC::RangeData& range, const long step) {
	std::cout << "ToJson data Range Data conversion...." << std::endl;
	std::stringstream ss;
	ss << "{\"base2laser\":[" << range.geometry.geometry.pose.position.x << "," << range.geometry.geometry.pose.position.y << "],"
		//ss << "{\"base2laser\":[" << 0.0 << ","
		//     << 0.0 << "],"    
		<< "\"amin\":" << range.config.minAngle << ","
		<< "\"amax\":" << range.config.maxAngle << ","
		<< "\"ares\":" << range.config.angularRes << ","
		<< "\"minRange\":" << range.config.minRange << ","
		<< "\"maxRange\":" << range.config.maxRange << ","
		<< "\"step\":" << step << ","
		<< "\"ranges\":[";
	const int len = range.ranges.length() / 4;
	for (int i = 0; i < len; i++) {
		ss << range.ranges[i*step];
		if (i != len - 1) ss << ",";
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
public:
	virtual ~HttpServer() {}


public:
	virtual void setRTC(NavigationManager* pRTC) = 0;
	virtual void initServer() = 0;
	virtual void runBackground(const std::string& baseDIr, const std::string& address, const int32_t port, const double timeout) = 0;


	virtual void terminateBackground() = 0;
};


HttpServer* createHttpServer();
