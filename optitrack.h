#pragma once

#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"
#include <memory>
#include <thread>
#include <unordered_map>
#include <string>
namespace optitrack{
	
	typedef std::shared_ptr<vrpn_Connection> ConnectionPtr;
	typedef std::shared_ptr<vrpn_Tracker_Remote> TrackerRemotePtr;
// 跟踪的对象实现
class VrpnTracker
{
public:
	typedef std::shared_ptr<VrpnTracker> Ptr;
	/**
	 * Create and initialize VrpnTrackerRos using an existing underlying VRPN connection object. The underlying
	 * connection object is responsible for calling the tracker's mainloop.
	 */
	VrpnTracker(const std::string& tracker_name, const ConnectionPtr& connection);
	/**
	 * Create and initialize VrpnTrackerRos, creating a new connection to tracker_name@host. This constructor will
	 * register timer callbacks on nh to call mainloop.
	 */
	VrpnTracker(std::string tracker_name, std::string host);
	~VrpnTracker();
	
	/**
	 * Call mainloop of underlying vrpn_Tracker_Remote
	 */
	void mainloop();

private:
	TrackerRemotePtr tracker_remote_;
	bool use_server_time_{},  process_sensor_id_{};
	std::string tracker_name;
	std::thread loop;
	bool isInvalidFirstCharInName(char c);
	static bool isInvalidSubsequentCharInName(char c);
	
	
	void init(std::string tracker_name);
	
	static void VRPN_CALLBACK handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose);
	
	static void VRPN_CALLBACK handle_twist(void *userData, const vrpn_TRACKERVELCB tracker_twist);
	
	static void VRPN_CALLBACK handle_accel(void *userData, const vrpn_TRACKERACCCB tracker_accel);
};


class optitrack_manager {
	typedef std::shared_ptr<optitrack_manager> Ptr;
	typedef std::unordered_map<std::string, VrpnTracker::Ptr> TrackerMap;
public:
	optitrack_manager();
	~optitrack_manager();
private:

	void manger_loop();
	void updateTrackers();
private:
	TrackerRemotePtr tracker_remote_;
	ConnectionPtr connection_;
	std::thread update,refresh_tracker;
   //Map of registered trackers, accessible by name
	TrackerMap trackers_;
};
}
