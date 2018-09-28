#ifndef KINCONTROLLER_H
#define KINCONTROLLER_H
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>
#include "ControllerConstants.h"


class KinController
{
public:
    KinController();
	KinController(bool publishTransform);
    ~KinController();
    bool statusOk();
    void spinOnce();
    
    void initializeNode();

    void updateState();
    bool broadcastUpdatedState();
    
    void updateVector(const geometry_msgs::Vector3Stamped& speed);
    
    ros::NodeHandle* ctrlNodeHandle;
    ros::Time current_time;
    ros::Time* speed_time;
    
    double speed_act_left = 0.0;
    double speed_act_right = 0.0;
    double speed_req1 = 0.0;
    double speed_req2 = 0.0;
    double speed_dt = 0.0;
    double x_pos = 0.0;
    double y_pos = 0.0;
    double theta = 0.0;

private:
    void updateDeltas();
    void updatePosition();
    void updateOrientation();
    void updateParameters();
    void configurePubSub();

    void getOdometryMessage(ros::Time current_time, geometry_msgs::Quaternion odometryQuaternion);
    void broadcastTransform(ros::Time current_time, 
        geometry_msgs::Quaternion odometryQuaternion);
    geometry_msgs::TransformStamped* getKinectTransform(ros::Time current_time);
    geometry_msgs::TransformStamped* getChassisTransform(ros::Time current_time, 
            geometry_msgs::Quaternion odometryQuaternion);
    
    tf::TransformBroadcaster* _broadcaster;
    ros::Duration* _duration;
    ros::Rate* _cycleRate;
    ros::NodeHandle* _nh_private;
    ros::Subscriber* _speedSub;
    ros::Publisher* _odometryPublisher;

    double _rate_hz = 10.0;
    double _linear_scale_positive = 1.0;
    double _linear_scale_negative = 1.0;
    double _angular_scale_positive = 1.0;
    double _angular_scale_negative = 1.0;
    bool _publishTransform = true;
    
    double _dt = 0.0;
    double _dth = 0.0;
    double _dxy = 0.0;
    double _vx = 0.0;
    double _vy = 0.0;
    double _vth = 0.0;
};

#endif // KINCONTROLLER_H
