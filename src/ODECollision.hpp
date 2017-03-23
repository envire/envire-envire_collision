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

#include "ODECollisionHeader.hpp"
#include "ODECollisionBase.hpp"
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <ode/collision.h>
#include <envire_core/items/ItemBase.hpp>


namespace envire { namespace collision
{

template <class T>
class ODECollision : public ODECollisionBase, public boost::noncopyable
{
public:
    typedef T TemplateType;

    dGeomID createNewCollisionObject(const boost::shared_ptr<T> user_data)
    {
        registerGeomClass();
        dGeomID geom = dCreateGeom(class_id);
        boost::shared_ptr<T>* _user_data = static_cast< boost::shared_ptr<T>* >(dGeomGetClassData(geom));
        *_user_data = user_data;
        return geom;
    }

    dGeomID createNewCollisionObject(const envire::core::ItemBase::Ptr user_data)
    {
        boost::shared_ptr<T> user_data_ = boost::dynamic_pointer_cast<T>(user_data);
        if(user_data_.get() == NULL)
            throw DownCastException<T>(user_data);

        return createNewCollisionObject(user_data_);
    }

    boost::shared_ptr<T> getUserData(dGeomID o)
    {
        checkClassID(o);
        boost::shared_ptr<T> _user_data = *static_cast< boost::shared_ptr<T>* >(dGeomGetClassData(o));
        return _user_data;
    }

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

    virtual ~ODECollision() {}

protected:

    ODECollision() : ODECollisionBase() {};

    virtual void getAABB (dGeomID o, dReal aabb[6], const boost::shared_ptr<T>& _user_data) = 0;

    virtual int collide (dGeomID o1, dGeomID o2,
                         int flags, dContactGeom *contact, int skip, const boost::shared_ptr<T>& _user_data, int o2_class_id) = 0;

    virtual int aABBTest (dGeomID o1, dGeomID o2, dReal aabb[6], const boost::shared_ptr<T>& _user_data)
    {
        dReal aabb_o1[6];
        getAABB(o1, aabb_o1, _user_data);
        return aabbCollision(aabb_o1, aabb);
    }

    virtual void geomDtor (dGeomID o, boost::shared_ptr<T>& _user_data)
    {
        _user_data.reset();
    }

    virtual void registerGeomClass() = 0;
};

}}