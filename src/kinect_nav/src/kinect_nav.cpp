#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>
#include "KinController.h"
#include "ControllerConstants.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "kinect_nav_controller");
  
    bool publish_tf = true;
    KinController kCtrl(publish_tf);

    while(kCtrl.statusOk()){
        kCtrl.spinOnce();
    }
}
