#define GLM_ENABLE_EXPERIMENTAL
#include "Transformable.h"
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>


namespace sereno
{
    Transformable::Transformable(const Rectangle3f& defaultConf) : m_transMatrix(1.0f), m_rotateMtx(1.0f), 
                                                                   m_scale(1.0f), m_applyMatrix(1.0f), 
                                                                   m_applyTransformation(NULL)
    {
        m_defaultPos  = glm::vec3(defaultConf.x, defaultConf.y, defaultConf.z);
        m_defaultSize = glm::vec3(defaultConf.width, defaultConf.height, defaultConf.depth);

        setTransMatrix();
    }

    Transformable::Transformable(const Transformable& cpy)
    {
        *this = cpy;
    }

    Transformable& Transformable::operator=(const Transformable& cpy)
    {
        if(this != &cpy)
        {
            m_transMatrix      = cpy.m_transMatrix;
            m_rotate           = cpy.m_rotate;
            m_rotateMtx        = cpy.m_rotateMtx;
            m_scale            = cpy.m_scale;
            m_position         = cpy.m_position;
            m_positionOrigin   = cpy.m_positionOrigin;
            m_applyMatrix      = cpy.m_applyMatrix;
            m_defaultSize      = cpy.m_defaultSize;
            m_defaultPos       = cpy.m_defaultPos;
            m_defaultPosOrigin = cpy.m_defaultPosOrigin;

            setApplyTransformation(cpy.m_applyTransformation);
        }

        return *this;
    }

    Transformable::~Transformable()
    {
        while(m_childrenTrans.size())
            if(m_childrenTrans[0])
                m_childrenTrans[0]->setApplyTransformation(NULL);
        if(m_applyTransformation)
            m_applyTransformation->removeTransfChild(this);
    }

    void Transformable::move(const glm::vec3 &v)
    {
        m_position += v;
        setTransMatrix();
    }

    void Transformable::setPositionOrigin(const glm::vec3 &p)
    {
        m_positionOrigin = -p;
        setTransMatrix();
    }

    void Transformable::setPosition(const glm::vec3 &v)
    {
        m_position = v;
        setTransMatrix();
    }

    void Transformable::setRotate(const Quaternionf& qRot)
    {
        m_rotate    = qRot;
        m_rotateMtx = qRot.getMatrix();
        setTransMatrix();
    }

    void Transformable::scale(const glm::vec3 &v)
    {
        m_scale = v;
        setTransMatrix();
    }

    void Transformable::setScale(const glm::vec3 &v)
    {
        scale(v);
    }

    void Transformable::setDefaultPos(const glm::vec3 &p)
    {
        m_defaultPos = p;
        setTransMatrix();
    }

    void Transformable::setDefaultSize(const glm::vec3 &s)
    {
        m_defaultSize = s;
        setTransMatrix();
    }

    void Transformable::setDefaultConf(const Rectangle3f &dc)
    {
        m_defaultPos  = glm::vec3(dc.x, dc.y, dc.z);
        m_defaultSize = glm::vec3(dc.width, dc.height, dc.depth);

        setTransMatrix();
    }

    void Transformable::setApplyTransformation(Transformable* transformable)
    {
        if(m_applyTransformation)
            m_applyTransformation->removeTransfChild(this);

        m_applyTransformation = transformable;
        if(m_applyTransformation)
            m_applyTransformation->addTransfChild(this);
    }

    void Transformable::removeTransfChild(Transformable* child)
    {
        for(std::vector<Transformable*>::iterator it = m_childrenTrans.begin(); it != m_childrenTrans.end(); it++)
            if(*it == child)
            {
                m_childrenTrans.erase(it);
                child->m_applyTransformation = NULL;
                child->resetChildrenTransMatrix();
                break;
            }
    }

    void Transformable::resetChildrenTransMatrix(const glm::mat4& mat)
    {
        m_applyMatrix = mat;
        glm::mat4 m = mat*m_transMatrix;
        for(std::vector<Transformable*>::iterator it = m_childrenTrans.begin(); it != m_childrenTrans.end(); it++)
            if(*it)
                (*it)->resetChildrenTransMatrix(m);
    }

    void Transformable::addTransfChild(Transformable* child)
    {
        if(child)
        {
            m_childrenTrans.push_back(child);
            child->resetChildrenTransMatrix(m_applyMatrix*m_transMatrix);
        }

    }

    const glm::vec3& Transformable::getScale() const
    {
        return m_scale;
    }

    glm::vec3 Transformable::getPosition() const
    {
        glm::vec3 v = m_position;
        v = v + m_defaultPos;
        return v;
    }

    const Quaternionf& Transformable::getRotate() const
    {
        return m_rotate;
    }

    glm::vec3 Transformable::getPositionOrigin() const
    {
        glm::vec3 v = -m_positionOrigin;
        return v;
    }

    const glm::vec3& Transformable::getDefaultPos() const
    {
        return m_defaultPos;
    }

    const glm::vec3& Transformable::getDefaultSize() const
    {
        return m_defaultSize;
    }

    Rectangle3f Transformable::getDefaultConf() const
    {
        return Rectangle3f(m_defaultPos, m_defaultSize);
    }

    glm::mat4 Transformable::getMatrix() const
    {
        if(m_applyTransformation)
            return m_applyMatrix * m_transMatrix;
        return m_transMatrix;
    }

    const glm::mat4& Transformable::getInternalMatrix() const
    {
        return m_transMatrix;
    }

    const Transformable* Transformable::getApplyTransformation() const
    {
        return m_applyTransformation;
    }

    Rectangle3f Transformable::mvpToRect(const glm::mat4& mvp) const
    {
        //Take the object 3D rect from its default configuration
        glm::vec4 v[8] = {
            glm::vec4(0.0, 0.0, 0.0, 1.0), glm::vec4(0.0, m_defaultSize[1], 0.0, 1.0), 
            glm::vec4(m_defaultSize[0], 0.0, 0.0, 1.0), glm::vec4(m_defaultSize[0], m_defaultSize[1], 0.0, 1.0), //Front face
            
            glm::vec4(0.0, 0.0, m_defaultSize[2], 1.0), glm::vec4(0.0, m_defaultSize[1], m_defaultSize[2], 1.0),
               glm::vec4(m_defaultSize[0], 0.0, m_defaultSize[2], 1.0), glm::vec4(m_defaultSize[0], m_defaultSize[1], m_defaultSize[2], 1.0)}; //Will be the back face

        for(uint32_t i=0; i < 8; i++)
        {
            //Add the default position
            v[i] = v[i] + glm::vec4(m_defaultPos, 0.0);
            //Then Calculate the transformation to these vec
            v[i] = mvp * v[i];
        }
        
        //Determine the maximum and minimum coord of the v[i] table
        float xMin, yMin, zMin, xMax, zMax, yMax;
        xMin = xMax = v[0][0];
        yMin = yMax = v[0][1];
        zMin = zMax = v[0][2];
        for(uint32_t i=1; i < 8; i++)
        {
            if(v[i][0] < xMin)
                xMin = v[i][0];
            else if(v[i][0] > xMax)
                xMax = v[i][0];

            if(v[i][1] < yMin)
                yMin = v[i][1];
            else if(v[i][1] > yMax)
                yMax = v[i][1];

            if(v[i][2] < zMin)
                zMin = v[i][2];
            else if(v[i][2] > zMax)
                zMax = v[i][2];
        }

        return Rectangle3f(xMin, yMin, zMin, xMax - xMin, yMax - yMin, zMax - zMin);
    }

    Rectangle3f Transformable::getInnerRect(const glm::mat4& m) const
    {
        return mvpToRect(m*m_transMatrix);
    }

    Rectangle3f Transformable::getRect(const glm::mat4& m) const
    {
        /* Get the proper transformable matrix.*/
        return mvpToRect(m*getMatrix());
    }

    PositionOrigin Transformable::getDefaultPositionOrigin() const
    {
        return m_defaultPosOrigin;
    }

    void Transformable::setDefaultPositionOrigin(PositionOrigin p)
    {
        m_defaultPosOrigin = p;
        setTransMatrix();
    }

    void Transformable::setTransMatrix()
    {    
        glm::mat4 posM(1.0f);
        posM = glm::translate(posM, computeDefaultPositionOrigin() + m_positionOrigin*getScale() + m_position);
        
        m_transMatrix = posM * m_rotateMtx;
        m_transMatrix = glm::scale(m_transMatrix, m_scale);

        for(std::vector<Transformable*>::iterator it = m_childrenTrans.begin(); it != m_childrenTrans.end(); it++)
            if(*it)
                (*it)->resetChildrenTransMatrix(m_applyMatrix * m_transMatrix);
    }

    glm::vec3 Transformable::computeDefaultPositionOrigin()
    {
        glm::vec3 s = getScale();
        switch(m_defaultPosOrigin)
        {
            case BOTTOM_LEFT:
                return glm::vec3(-m_defaultPos.x*s.x, -m_defaultPos.y*s.y, 0.0f);

            case BOTTOM_RIGHT:
                return glm::vec3((-m_defaultPos.x - m_defaultSize.x)*s.x, -m_defaultPos.y*s.y, 0.0f);

            case TOP_LEFT:
                return glm::vec3(-m_defaultPos.x*s.x, (-m_defaultPos.y-m_defaultSize.y)*s.y, 1.0f);

            case TOP_RIGHT:
                return glm::vec3((-m_defaultPos.x-m_defaultSize.x)*s.x, (-m_defaultPos.y-m_defaultSize.y)*s.y, 0.0f);

            case CENTER:
                return glm::vec3((-m_defaultPos.x-m_defaultSize.x/2.0f)*s.x, (-m_defaultPos.y-m_defaultSize.y/2.0f)*s.y, 0.0f);
            
            case TOP_CENTER:
                return glm::vec3((-m_defaultPos.x-m_defaultSize.x/2.0f)*s.x, (-m_defaultPos.y-m_defaultSize.y)*s.y, 0.0f);

            case BOTTOM_CENTER:
                return glm::vec3((-m_defaultPos.x-m_defaultSize.x/2.0f)*s.x, (-m_defaultPos.y)*s.y, 0.0f);

            case CENTER_LEFT:
                return glm::vec3((-m_defaultPos.x - m_defaultSize.x/2.0f)*s.x, (-m_defaultSize.y/2.0f -m_defaultPos.y)*s.y, 0.0);

            case CENTER_RIGHT:
                return glm::vec3((-m_defaultPos.x-m_defaultSize.x)*s.x, (-m_defaultPos.y-m_defaultSize.y/2.0f)*s.y, 0.0f);

            default:
                return glm::vec3(0.0f, 0.0f, 0.0f);
        }
        return glm::vec3(0.0f, 0.0f, 0.0f);
    }

    void Transformable::setRequestSize(const glm::vec3& v)
    {
        const glm::vec3& ds = getDefaultSize();
        glm::vec3 s  = glm::vec3(v.x / ((ds.x != 0) ? ds.x : 1),
                                 v.y / ((ds.y != 0) ? ds.y : 1),
                                 v.z / ((ds.z != 0) ? ds.z : 1));
        setScale(s);

    }

    void lookAt(Transformable& out, const glm::vec3& tmp, const glm::vec3& pos, const glm::vec3& target, bool rh)
    {
        out.setPosition(pos);

        glm::vec3 forward = glm::normalize(target-pos);
        if(rh)
            forward *= -1.0f;
        glm::vec3 right   = glm::cross(glm::normalize(tmp), forward);
        glm::vec3 up      = glm::cross(forward, right);

        float qw, qx, qy, qz;

        float m00 = right.x,   m01 = right.y,   m02 = right.z;
        float m10 = up.x,      m11 = up.y,      m12 = up.z;
        float m20 = forward.x, m21 = forward.y, m22 = forward.z;

        float tr = m00 + m11 + m22;

        if(tr > 0) 
        { 
            float s = sqrt(tr+1.0) * 2; // s=4*qw 
            qw = 0.25 * s;
            qx = (m21 - m12) / s;
            qy = (m02 - m20) / s; 
            qz = (m10 - m01) / s; 
        }
        else if((m00 > m11)&&(m00 > m22)) 
        { 
            float s = sqrt(1.0 + m00 - m11 - m22) * 2; // s=4*qx 
            qw = (m21 - m12) / s;
            qx = 0.25 * s;
            qy = (m01 + m10) / s; 
            qz = (m02 + m20) / s; 
        }
        else if (m11 > m22) 
        { 
            float s = sqrt(1.0 + m11 - m00 - m22) * 2; // s=4*qy
            qw = (m02 - m20) / s;
            qx = (m01 + m10) / s; 
            qy = 0.25 * s;
            qz = (m12 + m21) / s; 
        }
        else 
        { 
            float s = sqrt(1.0 + m22 - m00 - m11) * 2; // s=4*qz
            qw = (m10 - m01) / s;
            qx = (m02 + m20) / s;
            qy = (m12 + m21) / s;
            qz = 0.25 * s;
        }

        Quaternionf q(qx, qy, qz, qw);
        q.normalize();
        for(int i = 1; i < 4; i++)
            q[i] *= -1;

        out.setRotate(q);
    }
}
