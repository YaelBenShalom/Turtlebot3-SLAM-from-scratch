#include <cmath>
#include <iostream>
#include "rigid2d.hpp"

using namespace rigid2d;

/********** Transform2D Class member functions **********/

/// Create an identity transformation
Transform2D::Transform2D() {
    trans_angle = 0.0;
    trans_vector.x = 0.0;
    trans_vector.y = 0.0;
}

/// create a transformation that is a pure translation
Transform2D::Transform2D(const Vector2D & trans){
    trans_angle = 0.0;
    trans_vector.x = trans.x;
    trans_vector.y = trans.y;
}

/// create a pure rotation
Transform2D::Transform2D(double radians){
    trans_angle = radians;
    trans_vector.x = 0.0;
    trans_vector.y = 0.0;
}

/// Create a transformation with a translational and rotational component
Transform2D::Transform2D(const Vector2D & trans, double radians){
    trans_angle = radians;
    trans_vector.x = trans.x;
    trans_vector.y = trans.y;
}

/// apply a transformation to a Vector2D
Vector2D Transform2D::operator()(Vector2D v) const {
    Vector2D v_trans;
    v_trans.x = v.x * cos(trans_angle) - v.y * sin(trans_angle) + trans_vector.x;
    v_trans.y = v.x * sin(trans_angle) + v.y * cos(trans_angle) + trans_vector.y;
    return v_trans;
}

/// invert the transformation
Transform2D Transform2D::inv() const {
    Vector2D inv_trans;
    inv_trans.x = -trans_vector.x * cos(trans_angle) - trans_vector.y * sin(trans_angle);
    inv_trans.y = trans_vector.x * sin(trans_angle) - trans_vector.y * cos(trans_angle);
    Transform2D new_trans = Transform2D(inv_trans, -trans_angle);
    return new_trans;
}

/// compose this transform with another and store the result 
Transform2D & Transform2D::operator*=(const Transform2D & rhs){
double rhs_x = rhs.trans_vector.x;
double rhs_y = rhs.trans_vector.y;
double rhs_angle = rhs.trans_angle;

double lhs_x = trans_vector.x;
double lhs_y = trans_vector.y;
double lhs_angle = trans_angle;

trans_vector.x = lhs_x + rhs_x * cos(lhs_angle) - rhs_y * sin(lhs_angle);
trans_vector.y = lhs_y + rhs_x * sin(lhs_angle) + rhs_y * cos(lhs_angle);
trans_angle = lhs_angle + rhs_angle;

return *this;
}

/// output a 2 dimensional vector as [xcomponent, ycomponent]
std::ostream & operator<<(std::ostream & os, const Vector2D & v){
    os << "[" << v.x << ", " << v.y << "]\n";
    return os;
}

/// input a 2 dimensional vector
std::istream & operator>>(std::istream & is, Vector2D & v){
    std::cout << "Enter x coordinate" << std::endl;
    is >> v.x;

    std::cout << "Enter y coordinate" << std::endl;
    is >> v.y;

    // is >> std::noskipws;
    // double c1 = is.peek();
    // double c2 = is.get();
    // double c3 = is.get();
    // is >> c1 >> std::ws >> c3;
    // std::cout << "The coordinates entered: " << c1 << ", " << c3 << "\n" << std::endl;
    return is;
}


/********** Outside of Class **********/

/// \brief should print a human readable version of the transform:
/// An example output:
/// dtheta (degrees): 90 dx: 3 dy: 5
/// \param os - an output stream
/// \param tf - the transform to print
std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
    os << "dtheta (degrees): " << tf.trans_vector.x << ", dx: " << tf.trans_vector.y << ", dy: " << tf.trans_angle << "\n";
    return os;
}

// /// \brief Read a transformation from stdin
// /// Should be able to read input either as output by operator<< or
// /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
// std::istream & operator>>(std::istream & is, Transform2D & tf){
//     Vector2D v;
//     double theta;

//     std::cout << "Enter theta transform" << std::endl;
//     is >> theta;

//     std::cout << "Enter x transform" << std::endl;
//     is >> v.x;

//     std::cout << "Enter y transform" << std::endl;
//     is >> v.y;

//     tf = Transform2D transform(v, angle);

//     is >> tf.trans_vector.x;
//     is >> tf.trans_vector.y;
//     is >> tf.trans_angle;
//     return is;
// }

/// multiply two transforms together, returning their composition
Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
    return lhs *= rhs;
  }

