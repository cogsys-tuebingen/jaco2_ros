#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "jaco_calib_table");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  moveit::planning_interface::MoveGroupInterface group_("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

//  while (ros::ok())
//  {
      int offset = 0; // distance away from robot
      double scale = 0.8; //jaco
      std::vector<moveit_msgs::CollisionObject> collision_objects;
      shape_msgs::SolidPrimitive primitive;

      /* A pose for the box (specified relative to frame_id) */
      geometry_msgs::Pose box_pose;

      moveit_msgs::CollisionObject collision_object;



      //table top
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 3 * scale;    //direction of arm, x, red
      primitive.dimensions[1] = 2 * scale;    //left right of arm, y, green
      primitive.dimensions[2] = 0.1 * scale;    //height, z

      /* A pose for the box (specified relative to frame_id) */
      box_pose.orientation.w = 1.0;
//      box_pose.position.x = (offset + 1) * scale;
//      box_pose.position.y =  0 * scale;
//      box_pose.position.z = (.6 + 0.5* primitive.dimensions[2]) * scale;
      //facing wall
//      box_pose.position.x = 0;
//      box_pose.position.y = -(0.5 * primitive.dimensions[1] - 0.1);
//      box_pose.position.z = 0;
      //facing room
      box_pose.position.x = 0;
      box_pose.position.y = (0.5 * primitive.dimensions[1] - 0.1);
      box_pose.position.z = 0;
      collision_object.id = "table_top";
      //          collision_object.header.frame_id = group_.getPlanningFrame();

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      collision_objects.push_back(collision_object);


      collision_objects.push_back(collision_object);
      ROS_INFO("Add an object into the world");

      planning_scene_interface_.applyCollisionObjects(collision_objects);
      for (int i = 0; i < planning_scene_interface_.getKnownObjectNames().size(); i++) {
          ROS_INFO_STREAM("asdf "<< planning_scene_interface_.getKnownObjectNames()[i]);
      }
      sleep(0.5);

}

