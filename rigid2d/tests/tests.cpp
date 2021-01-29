#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main()

#include "catch.hpp"
#include "../include/rigid2d/rigid2d.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <gtest/gtest.h>

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



// TEST_CASE( "Tests case for vector operators", "[vector_operator]" ) {

//     // rigid2d::Vector2D v;

//     std::string input = "3 4";
//     std::stringstream string_in(input);

//     std::string output = "[3 4]\n";
//     std::stringstream string_out;

    // string_in >> v;
    // string_out << v;

    // // SECTION( "resizing bigger changes size and capacity" ) {
    //     REQUIRE( string_in.str() == input );
    // // }
    // // SECTION( "resizing smaller changes size but not capacity" ) {
    //     REQUIRE( stringT_out.str() == output );
    // // }
// }



// TEST_CASE( "Tests struct Vector2D", "[vector]" ) {

//     rigid2d::Vector2D v;
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