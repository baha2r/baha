#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>

static const std::string LOGNAME = "target_tracking_example";

// Creating a Class for monitoring status of moveit servo
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    status_sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "target Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber status_sub_;
};

// Creating a Class for subscribing to pose messages
class FrameSubscriber
{
private:
  ros::Subscriber delta_pose_sub;
  void deltaposCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    delta_pose.pose = msg->pose;
    delta_pose.header = msg->header;
    last_message_time = msg->header.stamp;

  }


public:
  geometry_msgs::PoseStamped delta_pose;
  ros::Time last_message_time;
  FrameSubscriber(ros::NodeHandle& nh)
  {
    delta_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/target_frame_pose", 50, &FrameSubscriber::deltaposCallback, this);
  }

 };

/**
 * Starts PoseTracking based on an external frame of reference, when using the robot end-tool to command the servo
 * i.e robot_link_command_frame and the ee are the same.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(8);
  spinner.start();

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Create the pose tracker
  moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

  // Make a publisher for sending pose commands
  ros::Publisher target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1, true);
  
  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(nh, "status");
  
  
  // Create a subscriber for target pose
  FrameSubscriber fs(nh);

  // Wait for the first message from the subscriber
  ROS_INFO_STREAM_NAMED(LOGNAME, "Waiting for initial target pose message...");
  auto initial_pose_msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/target_frame_pose", nh);


  // Define tolerances and timeout
  Eigen::Vector3d lin_tol{ 0.00001, 0.00001, 0.00001 };
  double rot_tol = 0.000001;
  double target_pose_timeout = 0.1; // seconds

  // Get the current EE transform
  geometry_msgs::TransformStamped current_ee_tf;
  tracker.getCommandFrameTransform(current_ee_tf);

  // Convert it to a Pose
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "ceiling_arm_tool0";

  // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
  // waypoints
  tracker.resetTargetPose();

  // Run the pose tracking in a new thread
  std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol, target_pose_timeout] {
    tracker.moveToPose(lin_tol, rot_tol, target_pose_timeout);
  });

  ros::Rate loop_rate(500);
  bool motion_stopped = false;
  bool initial_pose_received = false;
  bool no_new_messages = false;

  while (ros::ok())
  {
    // Modify the pose target each cycle
    target_pose.pose = fs.delta_pose.pose;
    target_pose.header.stamp = ros::Time::now();
    // Publish target pose
    target_pose_pub.publish(target_pose);

    // Check if the target pose is consistent and twist command is non-zero
    //if (!motion_stopped && previous_target_pose.pose == target_pose.pose && latest_twist_cmd.twist.linear.x != 0.0)
    if (initial_pose_msg) {
    initial_pose_received = true;
    //ROS_INFO("Received initial_pose_msg at: %f", initial_pose_msg->header.stamp.toSec());
    } else {
    initial_pose_received = false;
      //ROS_WARN("Did not receive initial_pose_msg within the timeout");
    }

    double time_diff = ros::Time::now().toSec() - fs.last_message_time.toSec();
    if (time_diff > 0.1)
    {
      no_new_messages = true;
      ROS_INFO("No new messages received in the last %.2f seconds.", time_diff);
    }
    else
    {
      no_new_messages = false;
      ROS_INFO("Messages received within the last %.2f seconds.", time_diff);
    }
    //ROS_INFO("Initial_pose_received: %s", initial_pose_received ? "true" : "false");
    //ROS_INFO("No new messages: %s", no_new_messages ?  "true" : "false");

    /*ROS_INFO("Last message received at: %f", fs.last_message_time.toSec());
    ROS_INFO("Current time is : %f", ros::Time::now().toSec());
    ROS_INFO("Time difference between last and current: %f", ros::Time::now().toSec()-fs.last_message_time.toSec());*/

    if (initial_pose_received ==true && no_new_messages == true)
    {
      ROS_INFO("Initial pose received and no new messages received. Stopping motion.");
      //tracker.stopMotion();
      ros::shutdown();
    }
    

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Join the pose tracking thread
  tracker.stopMotion();
  move_to_pose_thread.join();

  return EXIT_SUCCESS;
}
