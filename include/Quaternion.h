#ifndef  QUATERNION_INC
#define  QUATERNION_INC

#define GLM_FORCE_RADIANS

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <cmath>

namespace sereno
{
    template <typename T>
    struct Quaternion
    {
        public:
            T w = T(1); /*!< W component (real part) */
            T x = T(0); /*!< i part*/
            T y = T(0); /*!< j part*/
            T z = T(0); /*!< k part*/

            /* \brief Constructor putting this Quaternion into an Identity state */
            Quaternion()
            {}

            /* \brief Constructor. Q = _w + _x*i + _y*j + _z*k
             * \param _w the real component
             * \param _x the i part
             * \param _y the j part
             * \param _z the k part */
            Quaternion(T _x, T _y, T _z, T _w) : w(_w), x(_x), y(_y), z(_z)
            {}

            /* \brief Constructor from euler angles in radians
             * \param roll the rotation along x axis 
             * \param pitch the rotation along y axis
             * \param yaw the rotation along z axis*/
            Quaternion(T pitch, T roll, T yaw)
            {
                T cy = cos(yaw * 0.5);
                T sy = sin(yaw * 0.5);
                T cr = cos(roll * 0.5);
                T sr = sin(roll * 0.5);
                T cp = cos(pitch * 0.5);
                T sp = sin(pitch * 0.5);

                w = cy * cr * cp + sy * sr * sp;
                x = cy * sr * cp - sy * cr * sp;
                y = cy * cr * sp + sy * sr * cp;
                z = sy * cr * cp - cy * sr * sp;
            }

            /* \brief Copy Constructor
             * \parma q the quaternion to copy */
            Quaternion(const Quaternion& q)
            {
                *this = q;
            }

            /* \brief Movement constructor.
             * \param q the parameter to move */
            Quaternion(Quaternion&& q)
            {
                *this = q;
            }

            /* \brief Assignment operator
             * \parma q the Quaternion to copy */
            Quaternion& operator=(const Quaternion& q)
            {
                w = q.w;
                x = q.x;
                y = q.y;
                z = q.z;

                return *this;
            }

            /* \brief Constructor. Defines a quaternion via an axis and a rotation angle around it
             * \param axis the rotation axis
             * \param angle the rotation angle */
            Quaternion(const glm::tvec3<T>& axis, T angle)
            {
                T s = sin(angle/2);
                w   = cos(angle/2);
                x   = axis->x * s;
                y   = axis->y * s;
                z   = axis->z * s;
            }

            /* \brief Convert a quaternion into a rotation matrix
             * \return the rotation matrix */
            glm::tmat4x4<T> getMatrix() const
            {
                T xx = x*x;
                T yy = y*y;
                T zz = z*z;
                T wxT = 2*w*x;
                T wyT = 2*w*y;
                T wzT = 2*w*z;
                T xyT = 2*x*y;
                T xzT = 2*x*z;
                T yzT = 2*y*z;

                return glm::tmat4x4<T>(1 - 2*yy - 2*zz, xyT + wzT,     xzT - wyT,     0,
                                       xyT - wzT,       1-2*xx - 2*zz, yzT + wxT,     0,
                                       xzT + wyT,       yzT - wxT,     1-2*xx - 2*yy, 0,
                                       0,               0,             0,             1);

//                return glm::tmat4x4<T>(1 - 2*yy - 2*zz, xyT - wzT,     xzT + wyT,     0,
//                                       xyT + wzT,       1-2*xx - 2*zz, yzT - wxT,     0,
//                                       xzT - wyT,       yzT + wxT,     1-2*xx - 2*yy, 0,
//                                       0,               0,             0,             1);
            }
    };

    /* \brief perform Quaternion * Quaternion 
     * \param q1 the left operand quaternion
     * \param q2 the right operand quaternion
     * \return q3 with q3 = q1 * q2*/
    template <typename T>
    Quaternion<T> operator*(const Quaternion<T>& q1, const Quaternion<T>& q2)
    {
        return Quaternion<T>(q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,  //i part
                             q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,  //j part
                             q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w,  //k part
                             q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z); //The real part
    }

    typedef Quaternion<float>  Quaternionf;
    typedef Quaternion<double> Quaterniond;
}

#endif
