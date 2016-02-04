#pragma once

#include <base/Singleton.hpp>
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

