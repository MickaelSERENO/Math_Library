#ifndef DEF_TRANSFORMABLE
#define DEF_TRANSFORMABLE

#ifndef PI
#define PI 3.14159
#endif

#ifndef GLM_FORCE_RADIANS
#define GLM_FORCE_RADIANS
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <cmath>
#include <vector>
#include "Quaternion.h"
#include "Rectangle3.h"

namespace sereno
{
    /** \brief define a default origin position.*/
    enum PositionOrigin
    {
        NO_POSITION, TOP_LEFT, TOP_CENTER, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT, BOTTOM_CENTER,
        CENTER, CENTER_LEFT, CENTER_RIGHT
    };

    /** \brief Transformable is the class which manage all the transformation (rotations, translations, scales) of a matrix. It is mainly used for all the drawable and the Camera.
     * Firstly we apply the rotation, then the scale and after the rotation of the object.
     * The onScale, onRotate and onMove do nothing for this class. They are used if you want to do something during a transformation (without recreate move, scale or rotate function)*/
    class Transformable
    {
        public:
            /** \brief create an unity matrix
             * \param defaultConf the default size and position of this transformable */
            Transformable(const Rectangle3f& defaultConf=Rectangle3f(0, 0, 0, 0, 0, 0));

            ~Transformable();

            /** \brief translate the transformable
             * \param v the vector from where the object must be translated
             * \param useScale tell if the move statement should take care of the actual scale
             * */
            virtual void move(const glm::vec3 &v);

            /** \brief Set the position of the object
             * \param v the new position of the object
             * \param useScale Because of the scale, the position of the object will be multiply by the object scale. That's why, if you don't want this effect, you must set this parameter to false for dividing the position and get the real one (considering the scale) */
            virtual void setPosition(const glm::vec3 &v);

            /** \brief set the position origin of the transformable (another matrix which set the position of the position matrix)
             * \param p the new position origin
             * \param useScale Same as setPosition param*/
            virtual void setPositionOrigin(const glm::vec3 &p);

            /** \brief set the rotation of the object
             * \param qRot the quaternion rotation to apply*/
            virtual void setRotate(const Quaternionf& qRot);

            /** \brief scale the object
             * \param v values of the scale (x, y, z)
             * \param keepPos Cancel the scale set position (true) or not (false)*/
            virtual void scale(const glm::vec3 &v);

            /** \brief set the scale the object
             * \param v values of the scale (x, y, z)
             * \param keepPos Cancel the scale set position (true) or not (false)*/
            virtual void setScale(const glm::vec3 &v);

            /** \brief set the applyTransformation. 
             * The apply transformation is a variable telling which transformation should be used for the final computed matrix
             * \param transformable the new Transformable variable*/
            void setApplyTransformation(Transformable* transformable);

            /** \brief return the position of the object
             * \param useScale if set to true, the matrix position will be multiplied by the scale
             * \return the position of the object, considering or not the object scale*/
            virtual glm::vec3 getPosition() const;
            
            /* \brief Get the rotation of this transformable
             * \return a reference to the current Quaternion */
            virtual const Quaternionf& getRotate() const;

            /** \brief Get the origin position of this object
             * \return the position origin of this object*/
            virtual glm::vec3 getPositionOrigin() const;

            /** \brief return the size of the transformable with a scale at (1, 1, 1)
             * \return the size by default of the object*/
            virtual const glm::vec3& getDefaultSize() const;

            /** \brief return the default position of this transformable
             * \return the default position of the transformable*/
            virtual const glm::vec3& getDefaultPos() const;

            /** \brief get the default configuration of this transformable
             * \return the default configuration*/
            virtual Rectangle3f getDefaultConf() const;

            /** \brief get the scale of the object
             * \return value the scale value of the object of the x, y and z coordinates */
            virtual const glm::vec3& getScale() const;

            /** \brief get the result matrix of all the transformations (applyTransformation, position, scale, rotation)
             * \return the result matrix */
            virtual glm::mat4 getMatrix() const;

            /** \brief get the result matrix of all the transformations except applyTransformation (position, scale, rotation)
             * \return the internal matrix */
            const glm::mat4& getInternalMatrix() const;

            /** \brief get the applyTransformation. This variable is used if you want to transform he object only a few times with the same type of matrix
             * \return the apply transformation*/
            const Transformable* getApplyTransformation() const;

            /** \brief get the 3D rectangle on screen given its default configuration and the mvp applied
             * \param mvp The mvp matrix applied to the Transformable*/
            Rectangle3f mvpToRect(const glm::mat4& mvp) const;

            Rectangle3f getInnerRect(const glm::mat4& m=glm::mat4(1.0f)) const;

            /** \brief get the 3D rectangle of the transformable given its default position and default size
             * \param m if you want to apply a transformation before getting the 3D rectangle coordinates*/
            Rectangle3f getRect(const glm::mat4& m=glm::mat4(1.0f)) const;

            /** \brief set the default configuration of the object : default position and default size
             * \param dc the new defaultConf*/
            virtual void setDefaultConf(const Rectangle3f& dc);

            /** \brief add a position origin to the one defined by setPositionOrigin. Default : BOTTOM_LEFT 
             * \param p the default origin position*/
            void setDefaultPositionOrigin(PositionOrigin p);

            PositionOrigin getDefaultPositionOrigin() const;

            /** \brief This function is used for setting the size of the transformable. Used for complicated Drawable that doesn't want to scale bruptely the object entirely.
             * \param v the new size to apply. Basically, the size will be divised by the default size of the object. Remember that the Object is generally scaled. Use keepPos for keeping the transformable on the correct position*/
            virtual void setRequestSize(const glm::vec3& v);
        protected:
            /** \brief create the new matrix result from the position, the scale and the rotation. */
            void setTransMatrix();

            /** \brief set the default size of the object (the size when the scale is (1, 1, 1))
             * \param s the new default size*/
            virtual void setDefaultSize(const glm::vec3& s);

            /** \brief set the default position of the object
             * \param p the new default position*/
            virtual void setDefaultPos(const glm::vec3& p);

            /** \brief add a Transformable where trans->getApplyTransformation() == this.
             * \param trans the child to add*/
            void addTransfChild(Transformable* trans);

            /** \brief remove a transformation child into the children list of this Transformable
             * \param child the transformation child to remove  */
            void removeTransfChild(Transformable* child);

            /** \brief Recompute m_applyMatrix transformation matrix for each children
             * \param mat the parent matrix */
            void resetChildrenTransMatrix(const glm::mat4& mat = glm::mat4(1.0f));

            /* \brief Compute the default position origin of this Transformable based on m_defaultPosOrigin*/
            glm::vec3 computeDefaultPositionOrigin();

            glm::mat4                   m_transMatrix;                      /*!< The final computed transformation matrix*/
            Quaternionf                 m_rotate;                           /*!< The rotation quaternion. */
            glm::mat4                   m_rotateMtx;                        /*!< The resulting rotation matrix*/
            glm::vec3                   m_scale;                            /*!< The scaling factor*/
            glm::vec3                   m_position;                         /*!< The translation*/
            glm::vec3                   m_positionOrigin;                   /*!< The origin position*/
            glm::mat4                   m_applyMatrix;                      /*!< The sum of parent matrices */

            Transformable*              m_applyTransformation;              /*!< The parent transformation */
            std::vector<Transformable*> m_childrenTrans;                    /*!< The children transformation*/

            glm::vec3                   m_defaultSize;                      /*!< The default size of this transformation. Used to computed a bounding box*/
            glm::vec3                   m_defaultPos;                       /*!< The default position of this transformation. Used to computed a bounding box*/

            PositionOrigin              m_defaultPosOrigin = NO_POSITION;   /*!< Variable telling if the position origin is user-defined or anchors into one of the 9 points (top, center, left) x (left, center, right) coordinates*/
    };

    /* \brief  Function used principly by cameras to look at a particular object
     * \param out The transform to modify
     * \param up the upward vector
     * \param pos the transformable's new position
     * \param target the transformable's target position (where is it looking at?)
     * \param rh should the result be right-handed (true) or left-handed (false) ?*/
    void lookAt(Transformable& out, const glm::vec3& up, const glm::vec3& pos, const glm::vec3& target, bool rh);
}

#endif
