/// \file rigid2d.hpp
/// \brief Library for two-dimensional rigid body transformations.

#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP

#include <cmath>
#include <iosfwd>

namespace rigid2d
{
  /// \brief PI.  Not in C++ standard until C++20.
  constexpr double PI = 3.14159265358979323846;

  /// \brief approximately compare two floating-point numbers using
  ///        an absolute comparison
  /// \param d1 - a number to compare
  /// \param d2 - a second number to compare
  /// \param epsilon - absolute threshold required for equality
  /// \return true if abs(d1 - d2) < epsilon
  constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12)
  {
    if (fabs(d1 - d2) < epsilon)
    {
      return true;
    }
    return false;
  }

  /// \brief convert degrees to radians
  /// \param deg - angle in degrees
  /// \returns radians
  constexpr double deg2rad(double deg)
  {
    double rad = (deg / 360.0) * 2 * PI;
    return rad;
  }

  /// \brief convert radians to degrees
  /// \param rad - angle in radians
  /// \returns the angle in degrees
  constexpr double rad2deg(double rad)
  {
    double deg = (rad / (2 * PI)) * 360.0;
    return deg;
  }

  /// \brief turns any angle into the equivalent one between -PI and PI
  /// \param rad - angle in radians
  /// \returns the equivalent angle between -PI and PI
  constexpr double normalize_angle(double rad)
  {
    double deree = fmod(rad, 2 * PI);

    if (deree > PI)
    {
      deree -= 2 * PI;
    }
    else if (deree < -PI)
    {
      deree += 2 * PI;
    }
    return deree;
  }

  /// static_assertions test compile time assumptions.
  static_assert(almost_equal(0, 0), "almost_equal failed");
  static_assert(almost_equal(0.001, 0.005, 1.0e-2), "almost_equal failed");
  static_assert(almost_equal(0.1, 0.5, 1.0), "almost_equal failed");

  static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
  static_assert(almost_equal(deg2rad(180.0), PI), "deg2rad failed");

  static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");
  static_assert(almost_equal(rad2deg(PI), 180.0), "rad2deg failed");

  static_assert(almost_equal(deg2rad(rad2deg(0.0)), 0.0), "deg2rad failed");
  static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

  /// \brief A 2-Dimensional Vector
  struct Vector2D
  {
    /// \param x - x input of the vector
    double x;
    /// \param y - y input of the vector
    double y;
    /// \param x_normalized - x input of the vector - normalized
    double x_normalized;
    /// \param y_normalized - y input of the vector - normalized
    double y_normalized;

    /// \brief create zero-vector
    Vector2D();

    /// \brief create a vector with x,y inputs
    /// \param x_ - x input of the vector
    /// \param y_ - y input of the vector
    explicit Vector2D(double x_, double y_);

    /// \brief function to normalize vector
    /// \returns void
    void normalize();

    /// \brief compute the magnitude of the vector
    /// \param v - the vector
    /// \return the magnitude of the vector
    double magnitude(const Vector2D &v);

    /// \brief compute the angle of the vector
    /// \param v - the vector
    /// \return the angle of the vector
    double angle(const Vector2D &v);

    /// \brief add this vector with another and store the result
    /// in this object
    /// \param v - components to add
    /// \returns a reference to the newly transformed vector
    Vector2D &operator+=(const Vector2D &v);

    /// \brief subtract another vector from this vector and store the result
    /// in this object
    /// \param v - components to subtract
    /// \returns a reference to the newly transformed vector
    Vector2D &operator-=(const Vector2D &v);

    /// \brief multiply this vector with a scalar and store the result
    /// in this object
    /// \param scalar - scalar to multiply
    /// \returns a reference to the newly transformed vector
    Vector2D &operator*=(const double scalar);
  };

  /// \brief A 2-Dimensional twist
  struct Twist2D
  {
    /// \param thetadot - thetadot input of the vector
    double thetadot;
    /// \param xdot - xdot input of the vector
    double xdot;
    /// \param ydot - ydot input of the vector
    double ydot;

    /// \brief create a zero-Twist
    Twist2D();

    /// \brief create a twist with x,y inputs
    /// \param thetadot_ - thetadot input of the vector
    /// \param xdot_ - xdot input of the vector
    /// \param ydot_ - ydot input of the vector
    explicit Twist2D(double thetadot_, double xdot_, double ydot_);

    /// \brief \see operator>>(...) (declared outside this class)
    /// for a description.
    friend std::ostream &operator<<(std::ostream &os, const Twist2D &twist);

    /// \brief \see operator>>(...) (declared outside this class)
    /// for a description.
    friend std::istream &operator>>(std::istream &is, Twist2D &twist);
  };

  /// \brief a rigid body transformation in 2 dimensions
  class Transform2D
  {
  private:
    /// \param trans_vector - the vector by which to translate
    Vector2D trans_vector;
    /// \param trans_angle - the angle of the rotation, in radians
    double trans_angle;

  public:
    /// \brief Create an identity transformation
    Transform2D();

    /// \brief create a transformation that is a pure translation
    /// \param trans - the vector by which to translate
    explicit Transform2D(const Vector2D &trans);

    /// \brief create a pure rotation
    /// \param radians - the angle of the rotation, in radians
    explicit Transform2D(double radians);

    /// \brief Create a transformation with a translational and rotational
    /// component
    /// \param trans - the translation
    /// \param radians - the rotation, in radians
    Transform2D(const Vector2D &trans, double radians);

    /// \brief apply a transformation to a Vector2D
    /// \param v - the vector to transform
    /// \return a vector in the new coordinate system
    Vector2D operator()(Vector2D v) const;

    /// \brief apply a transformation to a Twist2D
    /// \param twist - the twist to transform
    /// \return a twist in the new coordinate system
    Twist2D operator()(Twist2D twist) const;

    /// \brief invert the transformation
    /// \return the inverse transformation.
    Transform2D inv() const;

    /// \brief compose this transform with another and store the result
    /// in this object
    /// \param rhs - the first transform to apply
    /// \returns a reference to the newly transformed operator
    Transform2D &operator*=(const Transform2D &rhs);

    /// \brief get the x displacement of the  transformation
    /// \return the x displacement
    double x() const;

    /// \brief get the y displacement of the  transformation
    /// \return the y displacement
    double y() const;

    /// \brief get the angular displacement of the transform
    /// \return the angular displacement
    double theta() const;

    /// \brief compute the transformation corresponding to a rigid body
    /// following a constant twist (in its original body frame) for one
    /// time unit
    /// \param twist - the twist to transform
    /// \return transformation correspond to a twist for one time step
    Transform2D integrateTwist(const Twist2D &twist) const;

    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    /// \returns ostream
    friend std::ostream &operator<<(std::ostream &os, const Transform2D &tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    /// \param is - stream from which to read
    /// \param tf - the transform to print
    /// \returns istream
    friend std::istream &operator>>(std::istream &is, Transform2D &tf);
  };

  /// \brief add this vector with another and store the result
  /// in this object
  /// \param v1 - vector
  /// \param v2 - vector to add
  /// \returns a reference to the newly transformed vector
  Vector2D &operator+(Vector2D v1, const Vector2D &v2);

  /// \brief subtract another vector from this vector and store the result
  /// in this object
  /// \param v1 - vector
  /// \param v2 - vector to subtruct
  /// \returns a reference to the newly transformed vector
  Vector2D &operator-(Vector2D v1, const Vector2D &v2);

  /// \brief multiply this vector with a scalar and store the result
  /// in this object
  /// \param v1 - vector vector to multiply
  /// \param scalar - scalar to multiply
  /// \returns a reference to the newly transformed vector
  Vector2D &operator*(Vector2D v1, const double scalar);

  /// \brief multiply a scalar with a vector and store the result
  /// in this object
  /// \param scalar - scalar to multiply
  /// \param v1 - vector vector to multiply
  /// \returns a reference to the newly transformed vector
  Vector2D &operator*(const double scalar, Vector2D v1);

  /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
  /// \param os - stream to output to
  /// \param v - the vector to print
  /// \returns ostream
  std::ostream &operator<<(std::ostream &os, const Vector2D &v);

  /// \brief input a 2 dimensional vector
  ///   You should be able to read vectors entered as two numbers
  ///   separated by a newline or a space, or entered as [xcomponent, ycomponent]
  /// \param is - stream from which to read
  /// \param v [out] - output vector
  /// \returns istream
  std::istream &operator>>(std::istream &is, Vector2D &v);

  /// \brief should print a human readable version of the twist:
  /// An example output:
  /// dtheta (degrees): 90 dx: 3 dy: 5
  /// \param os - an output stream
  /// \param twist - the twist to print
  /// \returns ostream
  std::ostream &operator<<(std::ostream &os, const Twist2D &twist);

  /// \brief Read a twist from stdin
  /// Should be able to read input either as output by operator<< or
  /// as 3 numbers (w, xdot, ydot) separated by spaces or newlines
  /// \param is - stream from which to read
  /// \param twist - the twist to print
  /// \returns istream
  std::istream &operator>>(std::istream &is, Twist2D &twist);

  /// \brief should print a human readable version of the transform:
  /// An example output:
  /// dtheta (degrees): 90 dx: 3 dy: 5
  /// \param os - an output stream
  /// \param tf - the transform to print
  /// \returns ostream
  std::ostream &operator<<(std::ostream &os, const Transform2D &tf);

  /// \brief Read a transformation from stdin
  /// Should be able to read input either as output by operator<< or
  /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
  /// \param is - stream from which to read
  /// \param tf - the transform to print
  /// \returns istream
  std::istream &operator>>(std::istream &is, Transform2D &tf);

  /// \brief multiply two transforms together, returning their composition
  /// \param lhs - the left hand operand
  /// \param rhs - the right hand operand
  /// \return the composition of the two transforms
  Transform2D operator*(Transform2D lhs, const Transform2D &rhs);
} // namespace rigid2d

#endif