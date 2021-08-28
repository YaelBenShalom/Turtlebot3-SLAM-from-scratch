#include "rigid2d/rigid2d.hpp"

#include <cmath>
#include <iostream>

/********** Vector2D struct member functions **********/

/// Create a 2-Dimensional Vector
rigid2d::Vector2D::Vector2D() {
  x = 0.0;
  y = 0.0;
}

/// Create a zero-vector
rigid2d::Vector2D::Vector2D(double x_, double y_) {
  x = x_;
  y = y_;
}

/// Normalize a vector
void rigid2d::Vector2D::normalize() {
  if (x == 0 && y == 0) {
    x_normalized = 0;
    y_normalized = 0;
  } else {
    x_normalized = x / sqrt(pow(x, 2) + pow(y, 2));
    y_normalized = y / sqrt(pow(x, 2) + pow(y, 2));
  }
}

/// Compute the magnitude of the vector
double rigid2d::Vector2D::magnitude(const rigid2d::Vector2D &v) {
  return std::sqrt(std::pow(v.x, 2) + std::pow(v.y, 2));
}

/// Compute the angle of the vector
double rigid2d::Vector2D::angle(const rigid2d::Vector2D &v) {
  return std::acos(v.x / rigid2d::Vector2D::magnitude(v));
}
/// Add this vector with another and store the result in this object
rigid2d::Vector2D &rigid2d::Vector2D::operator+=(const rigid2d::Vector2D &v) {
  x += v.x;
  y += v.y;
  // Vector2D::normalize();
  return *this;
}

/// Subtract another vector from this vector and store the result in this object
rigid2d::Vector2D &rigid2d::Vector2D::operator-=(const rigid2d::Vector2D &v) {
  x -= v.x;
  y -= v.y;
  // Vector2D::normalize();
  return *this;
}

/// Multiply this vector with a scalar and store the result in this object
rigid2d::Vector2D &rigid2d::Vector2D::operator*=(const double scalar) {
  x *= scalar;
  y *= scalar;
  // Vector2D::normalize();
  return *this;
}

/********** Twist2D class member functions **********/

/// Create a zero-Twist
rigid2d::Twist2D::Twist2D() {
  thetadot = 0.0;
  xdot = 0.0;
  ydot = 0.0;
}

/// Create a twist with x,y inputs
rigid2d::Twist2D::Twist2D(double thetadot_, double xdot_, double ydot_) {
  thetadot = thetadot_;
  xdot = xdot_;
  ydot = ydot_;
}

/********** Transform2D class member functions **********/

/// Create an identity transformation
rigid2d::Transform2D::Transform2D() {
  trans_angle = 0.0;
  trans_vector.x = 0.0;
  trans_vector.y = 0.0;
}

/// Create a transformation that is a pure translation
rigid2d::Transform2D::Transform2D(const Vector2D &trans) {
  trans_angle = 0.0;
  trans_vector.x = trans.x;
  trans_vector.y = trans.y;
}

/// Create a pure rotation
rigid2d::Transform2D::Transform2D(double radians) {
  trans_angle = radians;
  trans_vector.x = 0.0;
  trans_vector.y = 0.0;
}

/// Create a transformation with a translational and rotational component
rigid2d::Transform2D::Transform2D(const Vector2D &trans, double radians) {
  trans_angle = radians;
  trans_vector.x = trans.x;
  trans_vector.y = trans.y;
}

/// Apply a transformation to a Vector2D
rigid2d::Vector2D rigid2d::Transform2D::operator()(rigid2d::Vector2D v) const {
  Vector2D v_trans;

  // Calculating the transformation vector
  v_trans.x = v.x * cos(trans_angle) - v.y * sin(trans_angle) + trans_vector.x;
  v_trans.y = v.x * sin(trans_angle) + v.y * cos(trans_angle) + trans_vector.y;
  return v_trans;
}

/// Apply a transformation to a Twist2D
rigid2d::Twist2D
rigid2d::Transform2D::operator()(rigid2d::Twist2D twist) const {
  Twist2D twist_output;

  // Calculating the transformation swist
  twist_output.thetadot = twist.thetadot;
  twist_output.xdot = this->trans_vector.y * twist.thetadot +
                      cos(this->trans_angle) * twist.xdot -
                      sin(this->trans_angle) * twist.ydot;
  twist_output.ydot = -this->trans_vector.x * twist.thetadot +
                      sin(this->trans_angle) * twist.xdot +
                      cos(this->trans_angle) * twist.ydot;
  return twist_output;
}

/// Invert the transformation
rigid2d::Transform2D rigid2d::Transform2D::inv() const {
  Vector2D inv_trans;

  // Calculating the transformation inverse
  inv_trans.x =
      -trans_vector.x * cos(trans_angle) - trans_vector.y * sin(trans_angle);
  inv_trans.y =
      trans_vector.x * sin(trans_angle) - trans_vector.y * cos(trans_angle);
  Transform2D new_trans = rigid2d::Transform2D(inv_trans, -trans_angle);
  return new_trans;
}

/// Compose this transform with another and store the result in this object
rigid2d::Transform2D &
rigid2d::Transform2D::operator*=(const rigid2d::Transform2D &rhs) {
  double rhs_x = rhs.trans_vector.x;
  double rhs_y = rhs.trans_vector.y;
  double rhs_angle = rhs.trans_angle;

  double lhs_x = trans_vector.x;
  double lhs_y = trans_vector.y;
  double lhs_angle = trans_angle;

  // Calculating the transformation vector and angle
  trans_vector.x = lhs_x + rhs_x * cos(lhs_angle) - rhs_y * sin(lhs_angle);
  trans_vector.y = lhs_y + rhs_x * sin(lhs_angle) + rhs_y * cos(lhs_angle);
  trans_angle = lhs_angle + rhs_angle;
  return *this;
}

/// Get the x displacement of the transformation
double rigid2d::Transform2D::x() const {
  double x = trans_vector.x;
  return x;
}

/// Get the y displacement of the transformation
double rigid2d::Transform2D::y() const {
  double y = trans_vector.y;
  return y;
}

/// Get the angular displacement of the transform
double rigid2d::Transform2D::theta() const {
  double theta = trans_angle;
  return theta;
}

/// Compute the transformation corresponding to a rigid body following a
/// constant twist (in its original body frame) for one time unit
rigid2d::Transform2D
rigid2d::Transform2D::integrateTwist(const rigid2d::Twist2D &twist) const {
  double angle;
  Vector2D v;

  // If the theta component is 0
  if (twist.thetadot == 0.0) {
    v.x = twist.xdot;
    v.y = twist.ydot;
    rigid2d::Transform2D transform(v, 0);
    return transform;
  }
  // If the theta component is not 0
  else {
    angle = normalize_angle(twist.thetadot);
    v.x = std::sin(twist.thetadot) * twist.xdot / twist.thetadot +
          (std::cos(twist.thetadot) - 1) * twist.ydot / twist.thetadot;
    v.y = (1 - std::cos(twist.thetadot)) * twist.xdot / twist.thetadot +
          std::sin(twist.thetadot) * twist.ydot / twist.thetadot;
    rigid2d::Transform2D transform(v, angle);
    return transform;
  }
}

/********** Outside of Class - Vector2D related functions **********/

/// Add this vector with another and store the result in this object
rigid2d::Vector2D &operator+(rigid2d::Vector2D v1,
                             const rigid2d::Vector2D &v2) {
  return v1 += v2;
}

/// Subtract another vector from this vector and store the result in this object
rigid2d::Vector2D &operator-(rigid2d::Vector2D v1,
                             const rigid2d::Vector2D &v2) {
  return v1 -= v2;
}

/// Multiply this vector with a scalar and store the result in this object
rigid2d::Vector2D &operator*(rigid2d::Vector2D v1, const double scalar) {
  return v1 *= scalar;
}

/// Multiply a scalar with a vector and store the result in this object
rigid2d::Vector2D &operator*(const double scalar, rigid2d::Vector2D v1) {
  return v1 *= scalar;
}

/// Output a 2 dimensional vector as [x_component, y_component]
std::ostream &rigid2d::operator<<(std::ostream &os,
                                  const rigid2d::Vector2D &v) {
  os << "[" << v.x << ", " << v.y << "]\n";
  return os;
}

/// Input a 2 dimensional vector
std::istream &rigid2d::operator>>(std::istream &is, rigid2d::Vector2D &v) {
  std::cout << "Enter [x y] coordinates (m)" << std::endl;
  char c1 = is.peek();
  if (c1 == '[') {
    is.get();
  }

  is >> v.x >> v.y;
  std::cout << v.x << ", " << v.y << "\n" << std::endl;

  // v.normalize();
  return is;
}

/********** Outside of Class - Twist2D related functions **********/

/// Should print a human readable version of the twist
std::ostream &rigid2d::operator<<(std::ostream &os,
                                  const rigid2d::Twist2D &twist) {
  os << "thetadot (rad/s): " << twist.thetadot << " xdot (m/s): " << twist.xdot
     << " ydot (m/s): " << twist.ydot << std::endl;
  return os;
}

/// Read a twist from stdin
std::istream &rigid2d::operator>>(std::istream &is, rigid2d::Twist2D &twist) {
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
std::ostream &rigid2d::operator<<(std::ostream &os,
                                  const rigid2d::Transform2D &tf) {
  os << "dtheta (degrees): " << rad2deg(tf.trans_angle)
     << " dx: " << tf.trans_vector.x << " dy: " << tf.trans_vector.y
     << std::endl;
  return os;
}

/// Read a transformation from stdin
std::istream &rigid2d::operator>>(std::istream &is, rigid2d::Transform2D &tf) {
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
rigid2d::Transform2D rigid2d::operator*(Transform2D lhs,
                                        const rigid2d::Transform2D &rhs) {
  return lhs *= rhs;
}
