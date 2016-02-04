#include "ODECollision.hpp"

using namespace envire::collision;

ODECollisionBase::ODECollisionBase()
{
    class_id = -1;
}

dGeomID ODECollisionBase::createNewCollisionObject(const envire::core::ItemBase::Ptr user_data)
{
    throw CollisionPluginNotLinkedException(user_data);
}

envire::core::ItemBase::Ptr ODECollisionBase::getUserData(dGeomID o)
{
    checkClassID(o);
    envire::core::ItemBase::Ptr _user_data = *static_cast< envire::core::ItemBase::Ptr* >(dGeomGetClassData(o));
    return _user_data;
}

int ODECollisionBase::getGeomID()
{
    return class_id;
}

void ODECollisionBase::checkClassID(dGeomID o)
{
    int geom_id = dGeomGetClass(o);
    if(geom_id != this->class_id)
        throw WrongGeomClassID(this->class_id, geom_id);
}