#include "rigid2d/rigid2d.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <catch_ros/catch.hpp>


TEST_CASE( "Test function almost_equal(d1, d2, epsilon)" ) {

    REQUIRE( rigid2d::almost_equal(0, 0));
    REQUIRE( rigid2d::almost_equal(0.001, 0.005, 1.0e-2));
    REQUIRE( rigid2d::almost_equal(0.1, 0.5, 1.0));
    // REQUIRE( rigid2d::almost_equal(1, 500));  // This test should fail
}

TEST_CASE( "Test function deg2rad(deg)" ) {

    REQUIRE( rigid2d::almost_equal(rigid2d::deg2rad(0.0), 0.0));
    REQUIRE( rigid2d::almost_equal(rigid2d::deg2rad(180.0), rigid2d::PI));
    // REQUIRE( rigid2d::almost_equal(rigid2d::deg2rad(360.0), rigid2d::PI));  // This test should fail
}

TEST_CASE( "Test function rad2deg(rad)" ) {

    REQUIRE( rigid2d::almost_equal(rigid2d::rad2deg(0.0), 0.0));
    REQUIRE( rigid2d::almost_equal(rigid2d::rad2deg(rigid2d::PI), 180.0));
    // REQUIRE( rigid2d::almost_equal(rigid2d::rad2deg(rigid2d::PI), 360.0));  // This test should fail
}

TEST_CASE( "Test functions deg2rad(deg) and rad2deg(rad)" ) {

    REQUIRE( rigid2d::almost_equal(rigid2d::deg2rad(rigid2d::rad2deg(0.0)), 0.0));
    REQUIRE( rigid2d::almost_equal(rigid2d::deg2rad(rigid2d::rad2deg(2.1)), 2.1));
    // REQUIRE( rigid2d::almost_equal(rigid2d::rad2deg(180.0), 360.0));  // This test should fail
}

TEST_CASE( "Test Transform2D function inv()" ) {

    double angle = rigid2d::PI;
    rigid2d::Vector2D v;
    v.x = 1;
    v.y = 1;
    rigid2d::Transform2D T_ident;
    rigid2d::Transform2D T(v, angle);
    rigid2d::Transform2D T_inv = T.inv();

    REQUIRE( rigid2d::almost_equal((T * T_inv).x(), T_ident.x()));
    REQUIRE( rigid2d::almost_equal((T * T_inv).y(), T_ident.y()));
    REQUIRE( rigid2d::almost_equal((T * T_inv).theta(), T_ident.theta()));
}

TEST_CASE( "Test Transform2D function Transform2D(v, angle)" ) {

    double angle = 0.0;
    rigid2d::Vector2D v;
    v.x = 0.0;
    v.y = 0.0;
    rigid2d::Transform2D T_ident;
    rigid2d::Transform2D T(v, angle);

    REQUIRE( rigid2d::almost_equal(T.x(), T_ident.x()));
    REQUIRE( rigid2d::almost_equal(T.y(), T_ident.y()));
    REQUIRE( rigid2d::almost_equal(T.theta(), T_ident.theta()));
}

TEST_CASE( "Test Transform2D function integrateTwist(twist)" ) {

    rigid2d::Transform2D T, T_twisted;
    rigid2d::Twist2D twist;

    SECTION( "test pure translation" ) {
        twist.thetadot = 0;
        twist.xdot = 1;
        twist.ydot = 1;

        T_twisted = T.integrateTwist(twist);

        REQUIRE( rigid2d::almost_equal(T_twisted.x(), T.x() + 1));
        REQUIRE( rigid2d::almost_equal(T_twisted.y(), T.y() + 1));
        REQUIRE( rigid2d::almost_equal(T_twisted.theta(), T.theta()));
    }
    SECTION( "test pure rotation" ) {
        twist.thetadot = rigid2d::PI;
        twist.xdot = 0;
        twist.ydot = 0;

        T_twisted = T.integrateTwist(twist);

        REQUIRE( rigid2d::almost_equal(T_twisted.x(), T.x()));
        REQUIRE( rigid2d::almost_equal(T_twisted.y(), T.y()));
        REQUIRE( rigid2d::almost_equal(T_twisted.theta(), T.theta() + rigid2d::PI));
    }
    SECTION( "test simultaneous translation and rotation" ) {
        twist.thetadot = rigid2d::PI;
        twist.xdot = 1;
        twist.ydot = 1;

        T_twisted = T.integrateTwist(twist);

        REQUIRE( rigid2d::almost_equal(T_twisted.x(), T.x() - 0.63662, 1.0e-4));
        REQUIRE( rigid2d::almost_equal(T_twisted.y(), T.y() + 0.63662, 1.0e-4));
        REQUIRE( rigid2d::almost_equal(T_twisted.theta(), T.theta() + rigid2d::PI));
    }
}

TEST_CASE( "Test function normalize_angle(rad)" ) {

    double angle1 = rigid2d::PI * 4;
    double angle2 = 0;
    double angle3 = rigid2d::PI * 5;
    double angle4 = rigid2d::PI;
    double angle5 = rigid2d::PI * 4.5;
    double angle6 = rigid2d::PI/2;
    double angle7 = rigid2d::PI * 5.5;
    double angle8 = -rigid2d::PI/2;
    
    REQUIRE( rigid2d::almost_equal(rigid2d::normalize_angle(angle1), angle2));
    REQUIRE( rigid2d::almost_equal(rigid2d::normalize_angle(angle3), angle4));
    REQUIRE( rigid2d::almost_equal(rigid2d::normalize_angle(angle5), angle6));
    REQUIRE( rigid2d::almost_equal(rigid2d::normalize_angle(angle7), angle8));
}