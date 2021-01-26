#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main()

#include "catch.hpp"
#include "../include/rigid2d/rigid2d.hpp"

#include <cmath>
#include <iostream>
// #include <ros/ros.h>
#include <gtest/gtest.h>

using namespace rigid2d;

TEST_CASE( "Test function almost_equal(d1, d2, epsilon)" ) {
    REQUIRE( almost_equal(0, 0));
    REQUIRE( almost_equal(0.001, 0.005, 1.0e-2));
    REQUIRE( almost_equal(0.1, 0.5, 1.0));
    // REQUIRE( almost_equal(1, 500));  // This test should fail
}

TEST_CASE( "Test function deg2rad(deg)" ) {
    REQUIRE( almost_equal(deg2rad(0.0), 0.0));
    REQUIRE( almost_equal(deg2rad(180.0), PI));
    // REQUIRE( almost_equal(deg2rad(360.0), PI));  // This test should fail
}

TEST_CASE( "Test function rad2deg(rad)" ) {
    REQUIRE( almost_equal(rad2deg(0.0), 0.0));
    REQUIRE( almost_equal(rad2deg(PI), 180.0));
    // REQUIRE( almost_equal(rad2deg(PI), 360.0));  // This test should fail
}

TEST_CASE( "Test functions deg2rad(deg) and rad2deg(rad)" ) {
    REQUIRE( almost_equal(deg2rad(rad2deg(0.0)), 0.0));
    REQUIRE( almost_equal(deg2rad(rad2deg(2.1)), 2.1));
    // REQUIRE( almost_equal(rad2deg(180.0), 360.0));  // This test should fail
}


// TEST_CASE( "Tests struct Vector2D", "[vector]" ) {

//     std::vector<int> v( 5 );

//     REQUIRE( v.size() == 5 );
//     REQUIRE( v.capacity() >= 5 );

//     SECTION( "resizing bigger changes size and capacity" ) {
//         v.resize( 10 );

//         REQUIRE( v.size() == 10 );
//         REQUIRE( v.capacity() >= 10 );
//     }
//     SECTION( "resizing smaller changes size but not capacity" ) {
//         v.resize( 0 );

//         REQUIRE( v.size() == 0 );
//         REQUIRE( v.capacity() >= 5 );
//     }
//     SECTION( "reserving bigger changes capacity but not size" ) {
//         v.reserve( 10 );

//         REQUIRE( v.size() == 5 );
//         REQUIRE( v.capacity() >= 10 );
//     }
//     SECTION( "reserving smaller does not change size or capacity" ) {
//         v.reserve( 0 );

//         REQUIRE( v.size() == 5 );
//         REQUIRE( v.capacity() >= 5 );
//     }
// }