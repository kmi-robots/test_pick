// ROS
#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "sciroc_msgs/PerceptionAction.h"
#include "sciroc_msgs/DetectGrasps.h"

// MoveIt
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/move_group_interface/move_group_interface.h"

// TF2
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// PCL
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl/features/normal_3d.h"

// Eigen
#include "Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h"

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

class Pick {
public:
    explicit Pick(ros::NodeHandle &node) :
//    _gotCloud(false),
    _perc_client("image_action", true),
    _move_group("arm_torso") {
        _move_group.setPlanningTime(45.0);
        _grasp_client = node.serviceClient<sciroc_msgs::DetectGrasps>("detect_grasps");
//        _sub = node.subscribe("/xtion/depth_registered/points", 10, &Pick::pointcloudCallback, this);
        _pub = node.advertise<sensor_msgs::PointCloud2>("/depth_cloud", 10);
    }

//    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
//        _lastCloud = *msg;
//        _gotCloud = true;
//    }

    void addFloor()
    {
        _planning_scene_interface.removeCollisionObjects({"floor", "object"});
        // BEGIN_SUB_TUTORIAL table1
        //
        // Creating Environment
        // ^^^^^^^^^^^^^^^^^^^^
        // Create vector to hold 3 collision objects.
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(1);

        // Add the first table where the cube will originally be kept.
        collision_objects[0].id = "floor";
        collision_objects[0].header.frame_id = "base_footprint";

        /* Define the primitive and its dimensions. */
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions.resize(3);
        collision_objects[0].primitives[0].dimensions[0] = 1.0;
        collision_objects[0].primitives[0].dimensions[1] = 1.0;
        collision_objects[0].primitives[0].dimensions[2] = 0.05;

        /* Define the pose of the table. */
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0;
        collision_objects[0].primitive_poses[0].position.y = 0;
        collision_objects[0].primitive_poses[0].position.z = 0;
        collision_objects[0].primitive_poses[0].orientation.w = 1.0;
        // END_SUB_TUTORIAL

        collision_objects[0].operation = collision_objects[0].ADD;

        _planning_scene_interface.applyCollisionObjects(collision_objects);
    }

    void addObject(const Eigen::Quaternionf& rotation, const Eigen::Vector3f& position, std::vector<double> size) {
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(1);

        // Add the first table where the cube will originally be kept.
        collision_objects[0].id = "object";
        collision_objects[0].header.frame_id = "xtion_rgb_optical_frame";

        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions.resize(3);
        collision_objects[0].primitives[0].dimensions[0] = 0.9*size[0];
        collision_objects[0].primitives[0].dimensions[1] = 0.9*size[1];
        collision_objects[0].primitives[0].dimensions[2] = 0.9*size[2];

        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = position.x();
        collision_objects[0].primitive_poses[0].position.y = position.y();
        collision_objects[0].primitive_poses[0].position.z = position.z();

        collision_objects[0].primitive_poses[0].orientation.x = rotation.x();
        collision_objects[0].primitive_poses[0].orientation.y = rotation.y();
        collision_objects[0].primitive_poses[0].orientation.z = rotation.z();
        collision_objects[0].primitive_poses[0].orientation.w = rotation.w();

        collision_objects[0].operation = collision_objects[0].ADD;

        _planning_scene_interface.applyCollisionObjects(collision_objects);
    }

    static void closedGripper(trajectory_msgs::JointTrajectory& posture)
    {
        // BEGIN_SUB_TUTORIAL closed_gripper
        /* Add both finger joints of panda robot. */
        posture.joint_names.resize(2);
        posture.joint_names[0] = "gripper_left_finger_joint";
        posture.joint_names[1] = "gripper_right_finger_joint";

        /* Set them as closed. */
        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0.01;
        posture.points[0].positions[1] = 0.01;
        posture.points[0].time_from_start = ros::Duration(0.5);
        // END_SUB_TUTORIAL
    }

    void pick()
    {
        _planning_scene_interface.removeCollisionObjects({"floor", "object"});
        ROS_INFO("Waiting for perception action");
        _perc_client.waitForServer();

        sciroc_msgs::PerceptionGoal goal;
        goal.mode = 1;
        _perc_client.sendGoal(goal);

        bool finished_before_timeout = _perc_client.waitForResult(ros::Duration(30.0));

        if (!finished_before_timeout)
        {
            ROS_INFO("Action did not finish before the time out.");
            return;
        }

        actionlib::SimpleClientGoalState state = _perc_client.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        sciroc_msgs::PerceptionResult::ConstPtr result = _perc_client.getResult();

        ROS_INFO("Waiting for grasp server");
        _grasp_client.waitForExistence();

//        ROS_INFO("Waiting for cloud message");
//        while(!_gotCloud) {
//            ros::WallDuration(1.0).sleep();
//        }

        sciroc_msgs::DetectGrasps srv;
        srv.request.cloud = result->crop;

//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::fromROSMsg(result->crop, *cloud);
//        pcl::fromROSMsg(_lastCloud, *cloud);

//        passThroughFilter(cloud);
        // Compute point normals for later sample consensus step.
//        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//        computeNormals(cloud, cloud_normals);
        // inliers_plane will hold the indices of the point cloud that correspond to a plane.
//        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        // Detect and remove points on the (planar) surface on which the cylinder is resting.
//        removePlaneSurface(cloud, inliers_plane);
        // Remove surface points from normals as well
//        extractNormals(cloud_normals, inliers_plane);
//        sensor_msgs::PointCloud2 cloud_msg;
//        pcl::toROSMsg(*cloud, srv.request.cloud);
//        pcl::toROSMsg(*cloud, cloud_msg);
        _pub.publish(result->crop);

//        srv.request.cloud = cloud_msg;

        if (!_grasp_client.call(srv))
        {
            ROS_ERROR("Failed to call service detect_grasp");
            return;
        }
        std::vector<moveit_msgs::Grasp> grasps = srv.response.grasp_configs;


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(result->crop, *cloud);

        // Compute principal directions
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cloud, pcaCentroid);
        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

        // Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
        // Get the minimum and maximum points of the transformed cloud.
        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

        // Final transform
        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
        const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

        std::vector<double> v = {maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z};

        addObject(bboxQuaternion, bboxTransform, v);

        for(moveit_msgs::Grasp& g : grasps) {
            // Setting pre-grasp approach
            g.pre_grasp_approach.direction.header.frame_id = "arm_tool_link";
            g.pre_grasp_approach.direction.vector.x = 1.0;
            g.pre_grasp_approach.min_distance = 0.095;
            g.pre_grasp_approach.desired_distance = 0.2;

            // Setting post-grasp retreat
            g.post_grasp_retreat.direction.header.frame_id = "base_link";
            g.post_grasp_retreat.direction.vector.z = 1.0;
            g.post_grasp_retreat.min_distance = 0.1;
            g.post_grasp_retreat.desired_distance = 0.2;

            g.max_contact_force = 0.0;
            // BEGIN_SUB_TUTORIAL pick2
            // Setting posture of eef during grasp
            // +++++++++++++++++++++++++++++++++++
            closedGripper(grasps[0].grasp_posture);
            // END_SUB_TUTORIAL
        }

        // BEGIN_SUB_TUTORIAL pick3
        // Set support surface as table1.
        // Call pick to pick up the object using the grasps given
        _move_group.pick("object", grasps);
        // END_SUB_TUTORIAL
    }


private:
    ros::ServiceClient _grasp_client;
    actionlib::SimpleActionClient<sciroc_msgs::PerceptionAction> _perc_client;
    moveit::planning_interface::PlanningSceneInterface _planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface _move_group;
//    ros::Subscriber _sub;
    ros::Publisher _pub;
//    sensor_msgs::PointCloud2 _lastCloud;
//    bool _gotCloud;
};

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.05;
  posture.points[0].positions[1] = 0.05;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in verbose mode." This is a known issue. |br|
  // |br|
  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;

  Pick pick(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(5.0).sleep();

//  pick.addFloor();

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick.pick();

  ros::WallDuration(1.0).sleep();

//  place(group);

  ros::waitForShutdown();
  return 0;
}

// BEGIN_TUTORIAL
// CALL_SUB_TUTORIAL table1
// CALL_SUB_TUTORIAL table2
// CALL_SUB_TUTORIAL object
//
// Pick Pipeline
// ^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL pick1
// openGripper function
// """"""""""""""""""""
// CALL_SUB_TUTORIAL open_gripper
// CALL_SUB_TUTORIAL pick2
// closedGripper function
// """"""""""""""""""""""
// CALL_SUB_TUTORIAL closed_gripper
// CALL_SUB_TUTORIAL pick3
//
// Place Pipeline
// ^^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL place
// END_TUTORIAL
