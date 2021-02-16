#include "rigid2d/diff_drive.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <catch_ros/catch.hpp>
#include "ros/ros.h"



TEST_CASE( "Test DiffDrive constructors" ) {
    using namespace rigid2d;    
    auto wheel_base = 0.16;
    auto wheel_radious = 0.033;
    Config2D pose_start = {0,0,0};

    rigid2d::DiffDrive diff_drive(pose_start, wheel_base, wheel_radious);
    double right_angle, left_angle;
    left_angle = PI/2;
    right_angle = PI/2;
    WheelVelocity wheel_vel;
    WheelAngle wheel_angle;
    Config2D pose;
    wheel_vel = diff_drive.updateOdometryWithAngles(right_angle, left_angle);
    wheel_angle = diff_drive.get_wheel_angle();
    REQUIRE( rigid2d::almost_equal(wheel_angle.right_wheel_angle, right_angle));
    REQUIRE( rigid2d::almost_equal(wheel_angle.left_wheel_angle, left_angle));


    pose = diff_drive.get_config();
    REQUIRE( rigid2d::almost_equal(pose.x, PI/2*wheel_radious, 1e-3));
    REQUIRE( rigid2d::almost_equal(pose.y, 0));

    for(int i=0; i<50; i++){
    right_angle+=PI/2;
    left_angle+=PI/2;

    wheel_vel = diff_drive.updateOdometryWithAngles(right_angle, left_angle);
    wheel_angle = diff_drive.get_wheel_angle();
    REQUIRE( rigid2d::almost_equal(wheel_angle.right_wheel_angle, normalize_angle(right_angle)));
    REQUIRE( rigid2d::almost_equal(wheel_angle.left_wheel_angle,  normalize_angle(left_angle)));


    pose = diff_drive.get_config();
    ROS_INFO("pose.x = %f\n\r", pose.x);
    ROS_INFO("Reset robot position");
    ROS_INFO("Reset robot position");
    // REQUIRE( rigid2d::almost_equal(pose.x, 0.0518362, 1e-3));
    // REQUIRE( rigid2d::almost_equal(pose.y, 0));
    }
}
