#include "rigid2d.hpp" 

namespace rigid2d
{
    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
        private Vector2D trans_vector;

    public:
        /// \brief Create an identity transformation
        Transform2D() {
            trans_vector = {{1 0}, {0 1}};
        }

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const {
            Vector2D v_trans

            return 
        }

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const {
            double det = (trans_vector(0,0) * trans_vector(1,1)) - (trans_vector(1,0) * trans_vector(0,1));
            double inv_det = 1 / det;
            Transform2D new_trans = Transform2D();
            Vector2D inv_trans;
            inv_trans(0,0) = trans_vector(1,1) * inv_det;
            inv_trans(1,0) = -1 * trans_vector(1,0) * inv_det;
            inv_trans(0,1) = -1 * trans_vector(0,1) * inv_det;
            inv_trans(1,1) = trans_vector(0,0) * inv_det;
            new_trans = Transform2D(inv_trans);
            return new_trans;
        }


        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);
}
