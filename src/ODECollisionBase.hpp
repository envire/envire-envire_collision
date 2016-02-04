#pragma once

#include <ode/collision.h>
#include "Exceptions.hpp"
#include <Eigen/Geometry>

namespace envire { namespace collision
{


class ODECollisionBase
{
public:
    ODECollisionBase();

    virtual ~ODECollisionBase() {}

    virtual dGeomID createNewCollisionObject(const envire::core::ItemBase::Ptr user_data);

    envire::core::ItemBase::Ptr getUserData(dGeomID o);

    static void setTransformation(dGeomID o, const Eigen::Affine3d& transform)
    {
        Eigen::Matrix<dReal, 3, 1> translation = transform.translation().cast<dReal>();
        dGeomSetPosition(o, translation.x(), translation.y(), translation.z());
        Eigen::Quaternion<dReal> rotation(transform.rotation().cast<dReal>());
        dQuaternion quat;
        quat[0] = rotation.coeffs()[3];
        quat[1] = rotation.coeffs()[0];
        quat[2] = rotation.coeffs()[1];
        quat[3] = rotation.coeffs()[2];
        dGeomSetQuaternion(o, quat);
    }

    int getGeomID();

protected:

    void checkClassID(dGeomID o);

    static inline int aabbCollision(dReal aabb1[6], dReal aabb2[6])
    {
        if( aabb1[0] <= aabb2[1] &&
            aabb2[0] <= aabb1[1] &&
            aabb1[2] <= aabb2[3] &&
            aabb2[2] <= aabb1[3] &&
            aabb1[4] <= aabb2[5] &&
            aabb2[4] <= aabb1[5] )
            return 1;
        else
            return 0;
    }

protected:
    int class_id;
};

}}