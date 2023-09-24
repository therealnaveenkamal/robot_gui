#include "robotinfo_msgs/RobotInfo10Fields.h"
#define CVUI_IMPLEMENTATION
#include "nav_msgs/Odometry.h"
#include "robot_gui/cvui.h"
#include "std_srvs/Trigger.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <thread>

class CVUIROSCmdVelPublisher {
public:
  CVUIROSCmdVelPublisher();

  void run();

private:
  ros::Publisher twist_pub_;
  ros::Subscriber sub_;
  ros::Subscriber robot_info_sub_;

  geometry_msgs::Twist twist_msg;
  nav_msgs::Odometry data;
  robotinfo_msgs::RobotInfo10Fields robot_info_msg_;
  std::string twist_topic_name, odom_topic_name;
  float linear_velocity_step = 0.1;
  float angular_velocity_step = 0.1;
  const std::string WINDOW_NAME = "CVUI ROS TELEOP";
  void msgCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void
  robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);

  bool ros_thread_running;
};

CVUIROSCmdVelPublisher::CVUIROSCmdVelPublisher() {

  ros::NodeHandle nh1;
  odom_topic_name = "odom";
  sub_ = nh1.subscribe<nav_msgs::Odometry>(
      odom_topic_name, 2, &CVUIROSCmdVelPublisher::msgCallback, this);

  ros::NodeHandle nh2;
  robot_info_sub_ = nh2.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      "robot_info", 2, &CVUIROSCmdVelPublisher::robotInfoCallback, this);

  ros::NodeHandle nh;
  twist_topic_name = "cmd_vel";
  twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 10);
}

void CVUIROSCmdVelPublisher::msgCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  data = *msg;
}

void CVUIROSCmdVelPublisher::robotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_info_msg_ = *msg;
}

void CVUIROSCmdVelPublisher::run() {
  cv::Mat frame = cv::Mat(600, 350, CV_8UC3);
  std::string ros_distance = "0";
  ros::NodeHandle nh_gui;

  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = 0.0;

  ros::NodeHandle nh_service;
  ros::ServiceClient distance_client =
      nh_service.serviceClient<std_srvs::Trigger>("/get_distance");
  std_srvs::Trigger srv;

  std::thread ros_thread([this]() {
    ros::NodeHandle nh_ros;
    ros::Rate loop_rate(10);

    while (ros_thread_running && ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
  });

  ros::AsyncSpinner spinner(1);
  spinner.start();
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    frame = cv::Scalar(49, 52, 49);

    cvui::window(frame, 30, 5, 240, 180, "Info");
    cvui::printf(frame, 34, 25, 0.4, 0xffffff, "%s",
                 robot_info_msg_.data_field_01.c_str());
    cvui::printf(frame, 34, 40, 0.4, 0xffffff, "%s",
                 robot_info_msg_.data_field_02.c_str());
    cvui::printf(frame, 34, 55, 0.4, 0xffffff, "%s",
                 robot_info_msg_.data_field_03.c_str());
    cvui::printf(frame, 34, 70, 0.4, 0xffffff, "%s",
                 robot_info_msg_.data_field_04.c_str());

    cvui::printf(frame, 34, 85, 0.4, 0xffffff, "%s",
                 robot_info_msg_.data_field_05.c_str());

    cvui::printf(frame, 34, 100, 0.4, 0xffffff, "%s",
                 robot_info_msg_.data_field_06.c_str());

    cvui::printf(frame, 34, 115, 0.4, 0xffffff, "%s",
                 robot_info_msg_.data_field_07.c_str());

    cvui::printf(frame, 34, 130, 0.4, 0xffffff, "%s",
                 robot_info_msg_.data_field_08.c_str());

    cvui::printf(frame, 34, 145, 0.4, 0xffffff, "%s",
                 robot_info_msg_.data_field_09.c_str());
    cvui::printf(frame, 34, 160, 0.4, 0xffffff, "%s",
                 robot_info_msg_.data_field_10.c_str());

    twist_pub_.publish(twist_msg);
    if (cvui::button(frame, 100, 200, " Forward ")) {
      twist_msg.linear.x = twist_msg.linear.x + linear_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    if (cvui::button(frame, 100, 230, "   Stop  ")) {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
      twist_pub_.publish(twist_msg);
    }

    if (cvui::button(frame, 30, 230, " Left ")) {
      twist_msg.angular.z = twist_msg.angular.z + angular_velocity_step;
      twist_pub_.publish(twist_msg);
    }
    if (cvui::button(frame, 195, 230, " Right ")) {
      twist_msg.angular.z = twist_msg.angular.z - angular_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    if (cvui::button(frame, 100, 260, "Backward")) {
      twist_msg.linear.x = twist_msg.linear.x - linear_velocity_step;
      twist_pub_.publish(twist_msg);
    }
    cvui::window(frame, 30, 290, 110, 40, "Linear velocity:");
    cvui::printf(frame, 55, 315, 0.4, 0xff0000, "%.02f m/sec",
                 twist_msg.linear.x);
    cvui::window(frame, 150, 290, 110, 40, "Angular velocity:");
    cvui::printf(frame, 175, 315, 0.4, 0xff0000, "%.02f rad/sec",
                 twist_msg.angular.z);

    cvui::text(frame, 30, 340, "Estimated robot position based off odometry");

    cvui::window(frame, 30, 360, 80, 80, "X");
    cvui::printf(frame, 50, 390, 0.5, 0xffffff, "%.2f",
                 data.pose.pose.position.x);

    cvui::window(frame, 120, 360, 80, 80, "Y");
    cvui::printf(frame, 150, 390, 0.5, 0xffffff, "%.2f",
                 data.pose.pose.position.y);

    cvui::window(frame, 210, 360, 80, 80, "Z");
    cvui::printf(frame, 240, 390, 0.5, 0xffffff, "%.2f",
                 data.pose.pose.position.z);

    cvui::text(frame, 30, 480, "Distance Travelled");
    cvui::window(frame, 100, 500, 200, 50, "Distance in meters");
    cvui::printf(frame, 250, 525, 0.5, 0xffffff, "%.2f",
                 std::stof(ros_distance));

    if (cvui::button(frame, 30, 500, " Call ")) {
      if (distance_client.call(srv)) {
        if (srv.response.success) {
          ros_distance = srv.response.message;
        } else {
          ROS_ERROR("Service call failed: %s", srv.response.message.c_str());
        }
      } else {
        ROS_ERROR("Failed to call service");
      }
    }
    cvui::update();
    cv::imshow(WINDOW_NAME, frame);

    if (cv::waitKey(20) == 27) {
      ros_thread_running = false;
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");
  CVUIROSCmdVelPublisher button_clicks_publisher;
  ros::Rate loop_rate(10);
  while (ros::ok()) {

    button_clicks_publisher.run();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
