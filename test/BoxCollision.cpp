#include "BoxCollision.hpp"

using namespace envire::collision;

int BoxCollision::collide(dGeomID o1, dGeomID o2, int flags, dContactGeom* contact, int skip, const boost::shared_ptr< Eigen::AlignedBox3d >& _user_data, int o2_class_id)
{
    if(o2_class_id == this->class_id)
    {
        // transformations are ignored, since this is just a test class
        boost::shared_ptr< Eigen::AlignedBox3d > o2_user_data = getUserData(o2);
        Eigen::AlignedBox3d intersection = _user_data->intersection(*o2_user_data);
        if(!intersection.isEmpty())
        {
            if(flags == 1)
            {
                Eigen::Map< Eigen::Matrix<dReal, 4, 1> >(contact[0].pos).block(0,0,3,1) = intersection.center().cast<dReal>();
                contact[0].g1 = o1;
                contact[0].g2 = o2;
                return 1;
            }
            else if(flags > 1)
            {
                Eigen::Map< Eigen::Matrix<dReal, 4, 1> >(contact[0].pos).block(0,0,3,1) = intersection.min().cast<dReal>();
                contact[0].g1 = o1;
                contact[0].g2 = o2;
                Eigen::Map< Eigen::Matrix<dReal, 4, 1> >(contact[1].pos).block(0,0,3,1) = intersection.max().cast<dReal>();
                contact[1].g1 = o1;
                contact[1].g2 = o2;
                return 2;
            }
        }
    }
    else
    {
        // collision not implemented, therefore we can avoid to get called again
        dSetColliderOverride(this->class_id, o2_class_id, NULL);
    }
    return 0;
}

void BoxCollision::getAABB(dGeomID o, dReal aabb[6], const boost::shared_ptr< Eigen::AlignedBox3d >& _user_data)
{
    Eigen::Matrix<dReal, 3, 1> min = _user_data->min();
    Eigen::Matrix<dReal, 3, 1> max = _user_data->max();
    aabb[0] = min[0];
    aabb[1] = max[0];
    aabb[2] = min[1];
    aabb[3] = max[1];
    aabb[4] = min[2];
    aabb[5] = max[2];
}