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
            union
            {
                struct
                {
                    T w = T(1); /*!< W component (real part) */
                    T x = T(0); /*!< i part*/
                    T y = T(0); /*!< j part*/
                    T z = T(0); /*!< k part*/
                };
                T data[4];
            };

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
                for(int i = 0; i < 4; i++)
                    data[i] = q.data[i];

                return *this;
            }

            /* \brief Constructor. Defines a quaternion via an axis and a rotation angle around it
             * \param axis the rotation axis
             * \param angle the rotation angle */
            Quaternion(const glm::tvec3<T>& axis, T angle)
            {
                T s = sin(angle/2);
                w   = cos(angle/2);
                x   = axis.x * s;
                y   = axis.y * s;
                z   = axis.z * s;
            }

            T& operator[](int i)
            {
                return data[i];
            }

            const T& operator[](int i) const
            {
                return data[i];
            }

            void normalize() 
            {
                float mag = sqrt(x*x + y*y + z*z + w*w);
                w /= mag;
                x /= mag;
                y /= mag;
                z /= mag;
            }

            Quaternion<T> getInverse() const
            {
                return Quaternion<T>(-x, -y, -z, w);
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
            }

            /**
             * \brief  Rotate a vector by this quaternion
             * @tparam S
             * \param vec the vector to rotate
             * \return   the vector rotated
             */
            template <typename S>
            glm::tvec3<S> rotateVector(const glm::tvec3<S>& vec) const
            {
                Quaternion<T> invQ  = getInverse();
                Quaternion<T> qPure = Quaternion<T>(vec.x, vec.y, vec.z, 0);
//                Quaternion<T> res   = invQ * qPure * (*this);
                Quaternion<T> res   = (*this) * qPure * invQ;
                return glm::tvec3<S>(res.x, res.y, res.z);
            }

            /* \brief  Create a quaternion from euler angles
             * @tparam S
             * \param v the radian euler angles
             * \return   the quaternion created from euler angles */
            template <typename S>
            static Quaternion<T> fromEulerAngles(const glm::tvec3<S>& v)
            {
                double cy = cos(v.x * 0.5);
                double sy = sin(v.x * 0.5);
                double cr = cos(v.z * 0.5);
                double sr = sin(v.z * 0.5);
                double cp = cos(v.y * 0.5);
                double sp = sin(v.y * 0.5);

                Quaternion<T> q;

                q.w = cy * cr * cp + sy * sr * sp;
                q.x = cy * sr * cp - sy * cr * sp;
                q.y = cy * cr * sp + sy * sr * cp;
                q.z = sy * cr * cp - cy * sr * sp;

                return q;
            }

            /* \brief  Get the euler angle of this quaternion where you apply first x then y and finally z
             * \param rad do you want the result in radian?
             *
             * \return   the euler angles */
            glm::tvec3<T> toEulerAngles(bool rad=true) const
            {
                glm::tvec3<T> res;
                double test = x*y + z*w; //Use for the singularities
                if(test > 0.5 - 1.e-7)
                {
                    res.y = 2*atan2(x, w);
                    res.x = M_PI/2.0;
                    res.z = 0;
                }
                else if(test < -0.5 + 1.e-7)
                {
                    res.y = -2*atan2(x, w);
                    res.x = -M_PI/2.0;
                    res.z = 0;
                }
                else
                {
                    double sqx = x * x;
                    double sqy = y * y;
                    double sqz = z * z;
                    double h1  = 2*y*w - 2*x*z;
                    double h2  = 1 - 2*sqy - 2*sqz;
                    double b1  = 2*x*w-2*y*z;
                    double b2  = 1 - 2*sqx - 2*sqz;

                    res.z = atan2(b1, b2);
                    res.y = atan2(h1, h2);
                    res.x = asin(2*test);
                }

                //Change rad to degree if needed
                if(!rad)
                    for(int i = 0; i < 3; i++)
                        res[i] *= 180.0f/M_PI;

                return res;
            }

            T getMagnitude() const
            {
                return sqrt(x*x + y*y + z*z + w*w);
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

    template <typename T, typename S>
    glm::tvec3<S> operator*(const Quaternion<T>& q1, const glm::tvec3<S>& vec)
    {
        return q1.rotateVector(vec);
    }

    typedef Quaternion<float>  Quaternionf;
    typedef Quaternion<double> Quaterniond;
}

#endif
