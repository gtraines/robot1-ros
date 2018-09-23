#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>
#include "ControllerConstants.h"
#include "KinController.h"

KinController::KinController() {
    
    this->ctrlNodeHandle = new ros::NodeHandle(); //ros::NodeHandle n;
    this->_nh_private = new ros::NodeHandle("~"); //ros::NodeHandle nh_private_("~");
    
    this->initializeNode();
}

KinController::KinController(bool publishTransform) : this() {
    this->_publishTransform = publishTransform;
}

KinController::~KinController() {
    delete this->ctrlNodeHandle;
    delete this->_nh_private;
}

void KinController::initializeNode() {
    this->_duration(1.0);
    this->speed_time = new ros::Time(0.0);   
    this->configurePubSub();
    this->updateParameters();
}

bool KinController::statusOk() {
    return this->ctrlNodeHandle->ok();
}

void KinController::updateParameters() {
    this->_nh_private->getParam("publish_rate", this->_rate_hz);
    this->_nh_private->getParam("publish_tf", this->_publishTransform);
    this->_nh_private->getParam("linear_scale_positive", this->_linear_scale_positive);
    this->_nh_private->getParam("linear_scale_negative", this->_linear_scale_negative);
    this->_nh_private->getParam("angular_scale_positive", this->_angular_scale_positive);
    this->_nh_private->getParam("angular_scale_negative", this->_angular_scale_negative);
    
    this->_cycleRate(this->_rate_hz);
}

void KinController::updateVector(const geometry_msgs::Vector3Stamped& speed) {
    this->speed_act_left = trunc(speed.vector.x*100)/100;  //?
    ROS_INFO("speed left : %f", this->speed_act_left);
    this->speed_act_right = trunc(speed.vector.y*100)/100;
    ROS_INFO("speed right : %f", this->speed_act_right);
    this->speed_dt = speed.vector.z;
    this->speed_time = speed.header.stamp; // time stamp
}

void KinController::configurePubSub() {
    this->_broadcaster = new tf::TransformBroadcaster(); 
    this->_speedSub = &(this->ctrlNodeHandle->subscribe("speed", 50, this->updateVector)); //ros::Subscriber sub = n.subscribe("speed", 50, handle_speed);
    this->_odometryPublisher = n.advertise<nav_msgs::Odometry>("odom", 50);
}

void KinController::spinOnce() {
    ros::spinOnce();
    
    this->updateState();
    this->broadcastUpdatedState();

    this->_cycleRate.sleep();
}

void KinController::updateState() {
    this->updateDeltas();
    this->updatePosition();
    this->updateOrientation();
}

void KinController::updateDeltas() {
    double dt = this->speed_dt;					//Time in s
    this->_dxy = (this->speed_act_left+this->speed_act_right)*dt/2;
    this->_dth = ((this->speed_act_right-this->speed_act_left)*dt)/WHEELBASE_METERS;
    
    ROS_INFO("dt : %f", dt);
    ROS_INFO("dxy : %f", this->_dxy);
    
    if (this->_dth > 0) this->_dth *= this->_angular_scale_positive;
    if (this->_dth < 0) this->_dth *= this->_angular_scale_negative;
    if (this->_dxy > 0) this->_dxy *= this->_linear_scale_positive;
    if (this->_dxy < 0) this->_dxy *= this->_linear_scale_negative;

}

void KinController::updatePosition() {
    double dx = cos(this->_dth) * this->_dxy;
    double dy = sin(this->_dth) * this->_dxy;

    this->x_pos += (cos(this->theta) * dx - sin(this->theta) * dy);
    this->y_pos += (sin(this->theta) * dx + cos(this->theta) * dy);
}

void KinController::updateOrientation() {
    this->theta += this->_dth;

    // this is probably wrong if abs(delta_theta) can ever be > 2pi
    if(this->theta >= TWO_PI) this->theta -= TWO_PI;
    if(this->theta <= -TWO_PI) this->theta += TWO_PI;
}

bool KinController::broadcastUpdatedState() {
    ros::Time current_time = &(this->speed_time);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(this->theta);
    
    if (this->_publishTransform) {
        this->broadcastTransform(current_time, odom_quat);
    }
    
    auto odom_msg = this->getOdometryMessage(current_time, odom_quat);
    this->_odometryPublisher->publish(odom_msg);
}


bool KinController::broadcastTransform(ros::Time current_time, geometry_msgs::Quaternion odometryQuaternion) {
    geometry_msgs::TransformStamped kinectXform = this->getKinectTransform(current_time);
    geometry_msgs::TransformStamped chassisXform = this->getChassisTransform(current_time, odometryQuaternions);
    
    this->_broadcaster->sendTransform(kinectXform);
    this->_broadcaster->sendTransform(chassisXform);
}

geometry_msgs::TransformStamped KinController::getChassisTransform(ros::Time current_time, 
    geometry_msgs::Quaternion odometryQuaternion) {
        
    geometry_msgs::TransformStamped t;
    t.header.frame_id = ODOM;
    t.child_frame_id = BASE_LINK;
    t.transform.translation.x = this->x_pos;
    t.transform.translation.y = this->y_pos;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odometryQuaternion;
    t.header.stamp = current_time;
    
    return t;
}

geometry_msgs::TransformStamped* KinController::getKinectTransform(ros::Time current_time) {
    geometry_msgs::TransformStamped k;
    k.header.frame_id = KINECT_LINK;
    k.child_frame_id = CAMERA_LINK;
    k.transform.translation.x = 0.0;
    k.transform.translation.y = 0.0;
    k.transform.translation.z = 0.0;
    k.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    k.header.stamp = current_time;
    
    return k;
}

void KinController::getOdometryMessage(ros::Time current_time, geometry_msgs::Quaternion odometryQuaternion) {
    
    nav_msgs::Odometry odom_msg;
    
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = ODOM;
    odom_msg.pose.pose.position.x = this->x_pos;
    odom_msg.pose.pose.position.y = this->y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odometryQuaternion;
    
    if (this->speed_act_left == 0 && this->speed_act_right == 0){
      odom_msg.pose.covariance[0] = 1e-9;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 1e-9;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e-9;
      
      odom_msg.twist.covariance[0] = 1e-9;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 1e-9;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e-9;
    }
    else{
      odom_msg.pose.covariance[0] = 1e-3;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 0.0;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e3;
      
      odom_msg.twist.covariance[0] = 1e-3;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 0.0;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e3;
    }
    
    double vx = (this->speed_dt == 0) 
        ?  0 
        : (this->speed_act_left+this->speed_act_right)/2;
        
    double vth = (this->speed_dt == 0) 
        ? 0 
        : (this->speed_act_right-this->speed_act_left)/WHEELBASE_METERS;
        
    odom_msg.child_frame_id = BASE_LINK;
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = this->_dth;
    
    return odom_msg;
}
