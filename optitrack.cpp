
#include <sstream>
#include <iostream>
#include <algorithm>
#include "optitrack.h"
namespace optitrack {
	
	optitrack_manager::optitrack_manager() {
		
		std::stringstream host_stream;
		std::string server("192.168.12.85");
		host_stream << server;
		host_stream << ":" << 3883; // 端口号
		std::cout <<"Connecting to VRPN server at " << host_stream.str()<<std::endl;
		// 创建连接
		connection_ = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(host_stream.str().c_str()));
		if (connection_->connected()) {
			std::cout <<"Connection established"<<std::endl;
		}else {
			std::cout <<"Connection failed"<<std::endl;
		}
		refresh_tracker = std::thread(&optitrack_manager::updateTrackers, this);
		update = std::thread(&optitrack_manager::manger_loop, this);
	}
	
	optitrack::optitrack_manager::~optitrack_manager() {
		update.join();
		refresh_tracker.join();
	}
	void optitrack_manager::manger_loop()
	{
		while(true) {
			//std::cout <<" manger_loop " <<std::endl;
			connection_->mainloop();
			if (!connection_->doing_okay()) {
				std::cout << "WARN " << "VRPN connection is not 'doing okay'" << std::endl;
			}
			for (auto & tracker : trackers_) {
				tracker.second->mainloop();
			}
			// set 120HZ, sleep 8333 us
			std::this_thread::sleep_for(std::chrono::milliseconds (10));
		}
	}
	// Frequency at which to auto-detect new Trackers from the VRPN server. Mutually exclusive with ~trackers.
	void optitrack_manager::updateTrackers()
	{
		while(true) {
			//std::cout <<" updateTrackers " <<std::endl;
			int i = 0;
			while (connection_->sender_name(i) != NULL) {
				
				std::string name ="VRPN Control";
				std::string result = connection_->sender_name(i);
				if (trackers_.count(connection_->sender_name(i)) == 0 && result != name) {
					std::cout << "INFO " << "Found new sender: " << connection_->sender_name(i) << std::endl;
					trackers_.insert(std::make_pair(connection_->sender_name(i),
					                                std::make_shared<VrpnTracker>(connection_->sender_name(i),
					                                                              connection_)));
				}
				i++;
			}
			// set 10HZ, sleep 100 ms
			std::this_thread::sleep_for(std::chrono::milliseconds (100));
		}
	}
	bool VrpnTracker::isInvalidFirstCharInName(const char c)
	{
		return ! ( isalpha(c) || c == '/' || c == '~' );
	}
	bool VrpnTracker::isInvalidSubsequentCharInName(const char c)
	{
		return ! ( isalnum(c) || c == '/' || c == '_' );
	}
	
	VrpnTracker::VrpnTracker(const std::string& tracker_name, const ConnectionPtr& connection)
	{
		tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), connection.get());
		std::string clean_name = tracker_name;
		if (!clean_name.empty())
		{
			int start_subsequent = 1;
			if (isInvalidFirstCharInName(clean_name[0]))
			{
				clean_name = clean_name.substr(1);
				start_subsequent = 0;
			}
			clean_name.erase( std::remove_if( clean_name.begin() + start_subsequent, clean_name.end(), isInvalidSubsequentCharInName ), clean_name.end() );
		}
		init(clean_name);
	}
	VrpnTracker::~VrpnTracker()
	{
			loop.join();
			tracker_remote_->unregister_change_handler(this, &VrpnTracker::handle_pose);
	}
	void VrpnTracker::mainloop()
	{
		tracker_remote_->mainloop();
	}
	
	void VrpnTracker::init(std::string tracker_name)
	{
		std::cout << "INFO " <<"Found new sender: " <<"Creating new tracker " << tracker_name<< std::endl;
		tracker_remote_->register_change_handler(this, &VrpnTracker::handle_pose);
		tracker_remote_->shutup = true;
		this->tracker_name = tracker_name;
		std::string frame_id = "world"; // 当前位姿所在的坐标系
		use_server_time_ = false;       // 是不是使用服务器时间
		process_sensor_id_ = false;     // 当前的ID需不需要处理
	}
	void VRPN_CALLBACK VrpnTracker::handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose)
	{
		auto *tracker = static_cast<VrpnTracker *>(userData);
		std::size_t sensor_index(0);
		if (tracker->process_sensor_id_)
		{
			sensor_index = static_cast<std::size_t>(tracker_pose.sensor);
		}
		std::cout <<"ID: "<<sensor_index <<"time:" <<  tracker_pose.msg_time.tv_sec <<"+"<< tracker_pose.msg_time.tv_usec * 1000 ;
		std::cout <<"pose:"<< tracker_pose.pos[0]<<tracker_pose.pos[1]<<tracker_pose.pos[2]
		<< tracker_pose.quat[0]<<tracker_pose.quat[1]<<tracker_pose.quat[2]<<tracker_pose.quat[3]
		<< std::endl;
	}
	
} //namespace optitrack