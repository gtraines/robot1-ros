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
    void initializeNode();
    bool statusOk();
    void spinOnce();
    
    void updateState();
    bool broadcastUpdatedState();

    ros::NodeHandle* ctrlNodeHandle;
    ros::Time current_time;
    
    
    static void updateVector(const geometry_msgs::Vector3Stamped& speed);
    static ros::Time* speed_time;
    static double speed_act_left;
    static double speed_act_right;
    static double speed_dt;
    static double x_pos;
    static double y_pos;
    static double theta;

private:
    void updateDeltas();
    void updatePosition();
    void updateOrientation();
    void updateParameters();
    void configurePubSub();

    nav_msgs::Odometry getOdometryMessage(ros::Time current_time, geometry_msgs::Quaternion odometryQuaternion);
    void broadcastTransform(ros::Time current_time, geometry_msgs::Quaternion odometryQuaternion);
    geometry_msgs::TransformStamped getKinectTransform(ros::Time current_time);
    geometry_msgs::TransformStamped getChassisTransform(ros::Time current_time, geometry_msgs::Quaternion odometryQuaternion);
    
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
