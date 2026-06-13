#ifndef __BRING_UP_H__
#define __BRING_UP_H__
#include "ros/ros.h"


#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/reboot.h>

#include "tf/transform_broadcaster.h"

#include <mars_msgs/MarsStates.h>
#include <mars_msgs/MarsError.h>
#include <mars_msgs/HeartBeats.h>
#include <mars_msgs/DeviceOnoff.h>
#include <mars_msgs/DeviceAction.h>
#include <mars_msgs/DeviceStates.h>
#include <mars_msgs/ModuleStates.h>

class BringUp
{
public:
    BringUp();
    ~BringUp();

    void rosNode();
    void MapingModule();
    void sendAGVStates();
    // void StaticTFPublish();

    AresCmd *ares_cmd;

private:
    bool checkApplication(std::string appName);
    bool killApplication(std::string appName);

    void runMappingApp();
    void stopMappingApp();
    void runLocationApp();
    void stopLocationApp();
    void saveMapFile();

    bool checkHeartsBeats();
    void locationStateCallback();
    void motionStateCallback();
    void mappingStateCallback();

    bool handleAutoModeOnff();
    void subNavigaMode(const mars_msgs::MarsStates naviga_mode_stamsg);

private:
    FILE           *popen_check;
    FILE           *popen_cmdRun;

    mars_msgs::MarsStates   mars_state;
    mars_msgs::MarsError    mars_error;
    mars_msgs::HeartBeats   heart_beats;
    mars_msgs::DeviceStates device_state;
    mars_msgs::ModuleStates module_state;

    ros::NodeHandle         nh_;
    ros::Publisher   		mars_state_pub_;
    ros::Publisher   		mars_error_pub_;
    ros::Publisher   		heart_beats_pub_;
    ros::Publisher   		device_state_pub_;
    ros::Publisher   		module_state_pub_;

    ros::Subscriber         naviga_mode_sub_;
    // std::vector<double>         laser_vector3_;
    // std::vector<double>         laser_quaternion_;
    // tf::TransformBroadcaster    bc_laser2base;      // for laser -> base tf
    // tf::TransformBroadcaster    bc_scan2base;       // for scan  -> base tf

};
#endif
