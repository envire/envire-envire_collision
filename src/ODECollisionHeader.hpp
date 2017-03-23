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

#include <base-logging/Singleton.hpp>
#include <ode/collision.h>
#include <boost/shared_ptr.hpp>

#define ENVIRE_ODE_COLLISION_HEADER( classname ) \
public: \
    class ODEGeomRegistration ## classname \
    { \
    friend class classname; \
    private: \
        static int regGeomClass () \
        { \
            static dGeomClass geomClass ## classname; \
            geomClass ## classname.bytes = sizeof(boost::shared_ptr< classname::TemplateType >); \
            geomClass ## classname.collider = &dGetColliderFnFn; \
            geomClass ## classname.aabb = &dGetAABBFn; \
            geomClass ## classname.aabb_test = &dAABBTestFn; \
            geomClass ## classname.dtor = &dGeomDtorFn; \
            return dCreateGeomClass(&geomClass ## classname); \
        } \
        static void dGetAABBFn (dGeomID o, dReal aabb[6]) \
        { \
            classname* impl = classname::getInstance(); \
            impl->checkClassID(o); \
            boost::shared_ptr< classname::TemplateType > _user_data = impl->getUserData(o); \
            impl->getAABB(o, aabb, _user_data); \
        } \
        static int dCollideFn (dGeomID o1, dGeomID o2, \
                            int flags, dContactGeom *contact, int skip) \
        { \
            classname* impl = classname::getInstance(); \
            impl->checkClassID(o1); \
            boost::shared_ptr< classname::TemplateType > _user_data = impl->getUserData(o1); \
            return impl->collide(o1, o2, flags, contact, skip, _user_data, dGeomGetClass(o2)); \
        } \
        static dColliderFn * dGetColliderFnFn (int other_class_id) \
        { \
            return &dCollideFn; \
        } \
        static void dGeomDtorFn (dGeomID o) \
        { \
            classname* impl = classname::getInstance(); \
            impl->checkClassID(o); \
            boost::shared_ptr< classname::TemplateType > _user_data = impl->getUserData(o); \
            impl->geomDtor(o, _user_data); \
        } \
        static int dAABBTestFn (dGeomID o1, dGeomID o2, dReal aabb[6]) \
        { \
            classname* impl = classname::getInstance(); \
            impl->checkClassID(o1); \
            boost::shared_ptr< classname::TemplateType > _user_data = impl->getUserData(o1); \
            return impl->aABBTest(o1, o2, aabb, _user_data); \
        } \
    }; \
private: \
friend class ODEGeomRegistration ## classname; \
friend class base::Singleton< classname >; \
public: \
    static classname* getInstance() \
    { \
        return base::Singleton< classname >::getInstance(); \
    } \
private: \
    virtual void registerGeomClass() \
    { \
        if(class_id == -1) \
            class_id = classname::ODEGeomRegistration ## classname::regGeomClass(); \
    } \
    classname() : ODECollision() {}

