#pragma once

#include <envire_collision/ODECollision.hpp>
#include <Eigen/Geometry>

namespace envire { namespace collision
{

class BoxCollision : public ODECollision<Eigen::AlignedBox3d>
{
    ENVIRE_ODE_COLLISION_HEADER(BoxCollision)

protected:

    void getAABB (dGeomID o, dReal aabb[6], const boost::shared_ptr<Eigen::AlignedBox3d>& _user_data);

    int collide (dGeomID o1, dGeomID o2,
                         int flags, dContactGeom *contact, int skip, const boost::shared_ptr<Eigen::AlignedBox3d>& _user_data, int o2_class_id);

};

}}