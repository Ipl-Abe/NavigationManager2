

#include "HttpServer.h"

#define _WIN32_WINNT 0x0501
#include "httplib.h"
#include "NavigationManager.h"


class HttpServerImpl : public HttpServer {
private:
	httplib::Server server_;
	NavigationManager* m_pRTC;
public:
	virtual void setRTC(NavigationManager* pRTC) { m_pRTC = pRTC; }
public:
	HttpServerImpl() {}

	virtual void runBackground(const std::string& baseDir, const std::string& address, const int32_t port, const double timeoutSec) {
		server_.set_mount_point(nullptr, baseDir.c_str());

		thread_ = std::make_unique<std::thread>([address, port, this]() {
			server_.listen(address.c_str(), port);
			});
	}
	~HttpServerImpl() {
		terminateBackground();
		if (thread_) {
			thread_->join();
		}
	}

	virtual void terminateBackground() {
		server_.stop();
	}

	virtual void initServer() {
		std::string endpoint = "/api/";
		server_.Get((endpoint + "currentRobotPose").c_str(), [this](const httplib::Request& req, httplib::Response& res) {
			res.status = 200;
			res.version = "1.0";
			res.body = toJson(m_pRTC->getCurrentRobotPose());
			});

		server_.Get((endpoint + "rangeScan").c_str(), [this](const httplib::Request& req, httplib::Response& res) {
			res.status = 200;
			res.version = "1.0";
			res.body = toJson(m_pRTC->getRange(), 4);
			});

		server_.Put((endpoint + "map/refresh").c_str(), [this](const httplib::Request& req, httplib::Response& res) {
			if (m_pRTC->refreshMap()) {
				res.status = 200;
				res.version = "1.0";
				res.body = toJson(m_pRTC->getMapConfig());
			}
			else {
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
			res.body = toJson(m_pRTC->getMapConfig());
			});


		server_.Put((endpoint + "pf/refresh").c_str(), [this](const httplib::Request& req, httplib::Response& res) {
		  std::cout  << "/pf/refresh called" << std::endl;		  
			if (m_pRTC->refreshPF()) {
				std::lock_guard<std::mutex> g(m_pRTC->mcl_mutex_);
			  std::cout << " - OK. 200" << std::endl;
			  res.status = 200;
			  res.version = "1.0";
			  res.body = toJson(m_pRTC->getMCLInfo());
			  std::cout << " body: " << res.body << std::endl;
			}
			else {
			  std::cout << " - Failed. 200" << std::endl;			  
			  res.status = 402;
			  res.version = "1.0";
			  res.body = "Failed to load map from MapServer";
			}
		});

		server_.Get((endpoint + "pf/info").c_str(), [this](const httplib::Request& req, httplib::Response& res) {
		  res.status = 200;
		  res.version = "1.0";
		  res.body = toJson(m_pRTC->getMCLInfo());
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


HttpServer* createHttpServer() {
	return new HttpServerImpl();
}
