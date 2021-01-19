#include "rigid2d.hpp"
#include <cmath>

using namespace rigid2d;

// Transform2D member functions

/// \brief Create an identity transformation
Transform2D::Transform2D() {
    trans_angle = 0.0;
    trans_vector.x = 0.0;
    trans_vector.y = 0.0;
}

/// \brief create a transformation that is a pure translation
/// \param trans - the vector by which to translate
Transform2D::Transform2D(const Vector2D & trans){
    trans_angle = 0.0;
    trans_vector.x = trans.x;
    trans_vector.y = trans.y;
}

/// \brief create a pure rotation
/// \param radians - angle of the rotation, in radians
Transform2D::Transform2D(double radians){
    trans_angle = radians;
    trans_vector.x = 0.0;
    trans_vector.y = 0.0;
}

/// \brief Create a transformation with a translational and rotational
/// component
/// \param trans - the translation
/// \param rot - the rotation, in radians
Transform2D::Transform2D(const Vector2D & trans, double radians){
    trans_angle = radians;
    trans_vector.x = trans.x;
    trans_vector.y = trans.y;
}

/// \brief apply a transformation to a Vector2D
/// \param v - the vector to transform
/// \return a vector in the new coordinate system
Vector2D Transform2D::operator()(Vector2D v) const {
    Vector2D v_trans;
    v_trans.x = v.x * cos(trans_angle) - v.y * sin(trans_angle) + trans_vector.x;
    v_trans.y = v.x * sin(trans_angle) + v.y * cos(trans_angle) + trans_vector.y;
    return v_trans;
}

/// \brief invert the transformation
/// \return the inverse transformation. 
Transform2D Transform2D::inv() const {
    Vector2D inv_trans;
    inv_trans.x = -trans_vector.x * cos(trans_angle) - trans_vector.y * sin(trans_angle);
    inv_trans.y = trans_vector.x * sin(trans_angle) + trans_vector.y * cos(trans_angle);
    Transform2D new_trans = Transform2D(inv_trans, -trans_angle);
    return new_trans;
}

/// \brief compose this transform with another and store the result 
/// in this object
/// \param rhs - the first transform to apply
/// \returns a reference to the newly transformed operator
Transform2D & Transform2D::operator*=(const Transform2D & rhs){
double rhs_x = rhs.trans_vector.x;
double rhs_y = rhs.trans_vector.y;
double rhs_angle = rhs.trans_angle;

double lhs_x = this->trans_vector.x;
double lhs_y = this->trans_vector.y;
double lhs_angle = this->trans_angle;

Transform2D new_trans;
new_trans.trans_vector.x = lhs_x + rhs_x * cos(lhs_angle) - rhs_y * sin(lhs_angle);
new_trans.trans_vector.y = lhs_y + rhs_x * sin(lhs_angle) + rhs_y * cos(lhs_angle);
new_trans.trans_angle = lhs_angle + rhs_angle;

// Vector2D sol_trans;
// double sol_angle;
// sol_trans.x = lhs_x + rhs_x * cos(lhs_angle) - rhs_y * sin(lhs_angle);
// sol_trans.y = lhs_y + rhs_x * sin(lhs_angle) + rhs_y * cos(lhs_angle);
// sol_angle = lhs_angle + rhs_angle;
// Transform2D new_trans = Transform2D(sol_trans, sol_angle);

return new_trans;
}


// /// \brief \see operator<<(...) (declared outside this class)
// /// for a description
// friend std::ostream & Transform2D::operator<<(std::ostream & os, const Transform2D & tf);

// /// \brief should print a human readable version of the transform:
// /// An example output:
// /// dtheta (degrees): 90 dx: 3 dy: 5
// /// \param os - an output stream
// /// \param tf - the transform to print
// std::ostream & Transform2D::operator<<(std::ostream & os, const Transform2D & tf);

// /// \brief Read a transformation from stdin
// /// Should be able to read input either as output by operator<< or
// /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
// std::istream & Transform2D::operator>>(std::istream & is, Transform2D & tf);

// /// \brief multiply two transforms together, returning their composition
// /// \param lhs - the left hand operand
// /// \param rhs - the right hand operand
// /// \return the composition of the two transforms
// /// HINT: This function should be implemented in terms of *=
// Transform2D Transform2D::operator*(Transform2D lhs, const Transform2D & rhs);

