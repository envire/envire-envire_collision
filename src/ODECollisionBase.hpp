//
// Copyright (c) 2015, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

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