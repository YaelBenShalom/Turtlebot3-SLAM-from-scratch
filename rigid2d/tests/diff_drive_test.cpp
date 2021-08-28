#include "rigid2d/diff_drive.hpp"

#include "ros/ros.h"
#include <catch_ros/catch.hpp>
#include <cmath>
#include <iostream>
#include <sstream>

TEST_CASE("Test DiffDrive constructors") {
  auto wheel_base = 0.16;
  auto wheel_radius = 0.033;
  rigid2d::Config2D pose_start = {0, 0, 0};

  rigid2d::DiffDrive diff_drive(pose_start, wheel_base, wheel_radius);
  double right_angle, left_angle;
  left_angle = rigid2d::PI / 2;
  right_angle = rigid2d::PI / 2;
  rigid2d::WheelVelocity wheel_vel;
  rigid2d::WheelAngle wheel_angle;
  rigid2d::Config2D pose;

  wheel_vel = diff_drive.updateOdometryWithAngles(right_angle, left_angle);
  wheel_angle = diff_drive.get_wheel_angle();
  REQUIRE(rigid2d::almost_equal(wheel_angle.right_wheel_angle, right_angle));
  REQUIRE(rigid2d::almost_equal(wheel_angle.left_wheel_angle, left_angle));

  pose = diff_drive.get_config();
  REQUIRE(rigid2d::almost_equal(pose.x, rigid2d::PI / 2 * wheel_radius, 1e-3));
  REQUIRE(rigid2d::almost_equal(pose.y, 0));

  for (int i = 0; i < 50; i++) {
    right_angle += rigid2d::PI / 2;
    left_angle += rigid2d::PI / 2;

    wheel_vel = diff_drive.updateOdometryWithAngles(right_angle, left_angle);
    wheel_angle = diff_drive.get_wheel_angle();
    REQUIRE(rigid2d::almost_equal(wheel_angle.right_wheel_angle,
                                  rigid2d::normalize_angle(right_angle)));
    REQUIRE(rigid2d::almost_equal(wheel_angle.left_wheel_angle,
                                  rigid2d::normalize_angle(left_angle)));

    pose = diff_drive.get_config();
    ROS_INFO("pose.x = %f\n\r", pose.x);
    ROS_INFO("Reset robot position");
    ROS_INFO("Reset robot position");
  }
}
