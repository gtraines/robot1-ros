#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    // Initialize ROS, create the node handle and an async spinner
    ros::init(argc, argv, "move_group_add_object");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // We obtain the current planning scene and wait until everything is up
    // and running, otherwise the request won't succeed
    moveit::planning_interface::PlanningSceneInterface current_scene;

    sleep(5.0);

    // We create a box with certain dimensions and orientation, we also
    // give it a name which can later be used to remove it from the scene
    // The dimensions of the box (and also the object type which in this case
    // is box) is defined by a SolidPrimitive message, the pose of the box by a
    // pose message
    moveit_msgs::CollisionObject box;

    box.id = "rosbook_box";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.2;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x =  0.7;
    pose.position.y = -0.5;
    pose.position.z =  1.0;

    box.primitives.push_back(primitive);
    box.primitive_poses.push_back(pose);
    box.operation = box.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(box);

    // Once all of the objects (in this case just one) have been added to the
    // vector, we tell the planning scene to add our new box
    current_scene.addCollisionObjects(collision_objects);

    ros::shutdown();

    return 0;
}
