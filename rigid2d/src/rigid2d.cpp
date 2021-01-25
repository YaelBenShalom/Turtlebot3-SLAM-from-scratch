#include "../include/rigid2d/rigid2d.hpp"

#include <cmath>
#include <iostream>

using namespace rigid2d;

/********** Vector2D struct member functions **********/

/// Create a 2-Dimensional Vector
Vector2D::Vector2D() {
    x = 0.0;
    y = 0.0;
    Vector2D::normalize();
}

/// Create a zero-vector
Vector2D::Vector2D(double x_input, double y_input) {
    x = x_input;
    y = y_input;
    Vector2D::normalize();
}

/// Normalize a vector
void Vector2D::normalize() {
    if (x == 0 && y == 0){
        x_normalized = 0;
        y_normalized = 0;
    }
    else {
        x_normalized = x / sqrt(pow(x, 2) + pow(y, 2));
        y_normalized = y / sqrt(pow(x, 2) + pow(y, 2));
    }
}


/********** Twist2D class member functions **********/

/// Create a zero-Twist
Twist2D::Twist2D() {
    thetadot = 0.0;
    xdot = 0.0;
    ydot = 0.0;
}

/// Create a twist with x,y inputs
Twist2D::Twist2D(double thetadot_input, double xdot_input, double ydot_input) {
    thetadot = thetadot_input;
    xdot = xdot_input;
    ydot = ydot_input;
}


/********** Transform2D class member functions **********/

/// Create an identity transformation
Transform2D::Transform2D() {
    trans_angle = 0.0;
    trans_vector.x = 0.0;
    trans_vector.y = 0.0;
}

/// Create a transformation that is a pure translation
Transform2D::Transform2D(const Vector2D & trans) {
    trans_angle = 0.0;
    trans_vector.x = trans.x;
    trans_vector.y = trans.y;
}

/// Create a pure rotation
Transform2D::Transform2D(double radians) {
    trans_angle = radians;
    trans_vector.x = 0.0;
    trans_vector.y = 0.0;
}

/// Create a transformation with a translational and rotational component
Transform2D::Transform2D(const Vector2D & trans, double radians) {
    trans_angle = radians;
    trans_vector.x = trans.x;
    trans_vector.y = trans.y;
}

/// Apply a transformation to a Vector2D
Vector2D Transform2D::operator()(Vector2D v) const {
    Vector2D v_trans;
    v_trans.x = v.x * cos(trans_angle) - v.y * sin(trans_angle) + trans_vector.x;
    v_trans.y = v.x * sin(trans_angle) + v.y * cos(trans_angle) + trans_vector.y;
    return v_trans;
}

/// Apply a transformation to a Twist2D
Twist2D Transform2D::operator()(Twist2D twist) const {
    Twist2D twist_output;
    twist_output.thetadot = twist.thetadot;
    twist_output.xdot = this->trans_vector.y * twist.thetadot + cos(this->trans_angle) * twist.xdot - sin(this->trans_angle) * twist.ydot;
    twist_output.ydot = -this->trans_vector.x * twist.thetadot + sin(this->trans_angle) * twist.xdot + cos(this->trans_angle) * twist.ydot;
    return twist_output;
}

/// Invert the transformation
Transform2D Transform2D::inv() const {
    Vector2D inv_trans;
    inv_trans.x = -trans_vector.x * cos(trans_angle) - trans_vector.y * sin(trans_angle);
    inv_trans.y = trans_vector.x * sin(trans_angle) - trans_vector.y * cos(trans_angle);
    Transform2D new_trans = Transform2D(inv_trans, trans_angle);
    return new_trans;
}

/// Compose this transform with another and store the result 
Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
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


/********** Outside of Class - Vector2D related functions **********/

/// Output a 2 dimensional vector as [xcomponent, ycomponent]
std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v) {
    os << "[" << v.x << ", " << v.y << "]\n";
    return os;
}

/// Input a 2 dimensional vector
std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Vector2D & v) {
    std::cout << "Enter x coordinates (m)" << std::endl;
    is >> v.x;

    std::cout << "Enter y coordinates (m)" << std::endl;
    is >> v.y;
    
    // is >> std::noskipws;
    // double c1 = is.peek();
    // double c2 = is.get();
    // double c3 = is.get();
    // is >> c1 >> std::ws >> c3;
    // std::cout << "The coordinates entered: " << c1 << ", " << c3 << "\n" << std::endl;
    
    v.normalize();
    return is;
}


/********** Outside of Class - Twist2D related functions **********/

/// Should print a human readable version of the twist
std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Twist2D & twist) {
    os << "thetadot (rad/s): " << twist.thetadot
       << "\t xdot (m/s): " << twist.xdot
       << "\t ydot (m/s): " << twist.ydot << std::endl;
    return os;
}

/// Read a twist from stdin
std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Twist2D & twist) {
    std::cout << "Enter thetadot velocity (rad/s):" << std::endl;
	is >> twist.thetadot;

	std::cout << "Enter xdot velocity (m/s):" << std::endl;
	is >> twist.xdot;

	std::cout << "Enter ydot velocity (m/s):" << std::endl;
	is >> twist.ydot;

	return is;
}


/********** Outside of Class - Transform2D related functions **********/

/// Should print a human readable version of the transform
std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Transform2D & tf) {
    os << "dtheta (degrees): " << rad2deg(tf.trans_angle)
       << "\t dx: " << tf.trans_vector.x
       << "\t dy: " << tf.trans_vector.y << std::endl;
    return os;
}

/// Read a transformation from stdin
std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Transform2D & tf) {
    Vector2D v;
    double theta;

    std::cout << "Enter theta transform (degrees)" << std::endl;
    is >> theta;

    std::cout << "Enter x transform (m)" << std::endl;
    is >> v.x;

    std::cout << "Enter y transform (m)" << std::endl;
    is >> v.y;

    tf.trans_angle = deg2rad(theta);
    tf.trans_vector.x = v.x;
    tf.trans_vector.y = v.y;

    return is;
}

/// Multiply two transforms together, returning their composition
rigid2d::Transform2D rigid2d::operator*(rigid2d::Transform2D lhs, const rigid2d::Transform2D & rhs) {
    return lhs *= rhs;
  }