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

#include <boost/test/unit_test.hpp>
#include <iostream>
#include <ode/odeinit.h>
#include <envire_core/items/Item.hpp>
#include <envire_collision/Exceptions.hpp>

#include "BoxCollision.hpp"

using namespace envire::collision;

BOOST_AUTO_TEST_CASE(test_box_collision)
{
    dInitODE();
    {
        BoxCollision* c = BoxCollision::getInstance();
        BOOST_CHECK(c != NULL);
        BOOST_CHECK(c->getGeomID() == -1);

        boost::shared_ptr<Eigen::AlignedBox3d> user_data_a(new Eigen::AlignedBox3d);
        user_data_a->extend(Eigen::Vector3d(0,0,0));
        user_data_a->extend(Eigen::Vector3d(1,1,1));

        // create first geom
        dGeomID geom_a = c->createNewCollisionObject(user_data_a);
        c->setTransformation(geom_a, Eigen::Affine3d::Identity());

        // check interface
        BOOST_CHECK(c->getGeomID() >= dFirstUserClass);
        BOOST_CHECK(dGeomGetClass(geom_a) == c->getGeomID());

        boost::shared_ptr<Eigen::AlignedBox3d> user_data_a2 = c->getUserData(geom_a);

        BOOST_CHECK(user_data_a.use_count() == 3);
        BOOST_CHECK(user_data_a == user_data_a2);

        // create second geom
        boost::shared_ptr<Eigen::AlignedBox3d> user_data_b(new Eigen::AlignedBox3d);
        user_data_b->extend(Eigen::Vector3d(-0.5,-0.5,-0.5));
        user_data_b->extend(Eigen::Vector3d(0.5,0.5,0.5));
        dGeomID geom_b = c->createNewCollisionObject(user_data_b);
        c->setTransformation(geom_b, Eigen::Affine3d::Identity());

        BOOST_CHECK(geom_a != geom_b);

        // check collision
        // one point
        Eigen::AlignedBox3d intersection = user_data_a->intersection(*user_data_b);
        dContactGeom cg[1];
        int points = dCollide(geom_a, geom_b, 1,  &cg[0], sizeof cg[0]);
        BOOST_CHECK(points == 1);
        BOOST_CHECK((Eigen::Map< Eigen::Matrix<dReal, 3, 1> >(cg[0].pos).block(0,0,3,1) == intersection.center().cast<dReal>()));

        // two points
        dContactGeom cg2[10];
        points = dCollide(geom_a, geom_b, 10,  &cg2[0], sizeof cg2[0]);
        BOOST_CHECK(points == 2);
        BOOST_CHECK((Eigen::Map< Eigen::Matrix<dReal, 3, 1> >(cg2[0].pos).block(0,0,3,1) == intersection.min().cast<dReal>()));
        BOOST_CHECK((Eigen::Map< Eigen::Matrix<dReal, 3, 1> >(cg2[1].pos).block(0,0,3,1) == intersection.max().cast<dReal>()));

        // no collision
        user_data_b->setEmpty();
        points = dCollide(geom_a, geom_b, 10,  &cg2[0], sizeof cg2[0]);
        BOOST_CHECK(points == 0);

        // not implemented collision
        dGeomID sphere = dCreateSphere(0, 2.0);
        c->setTransformation(sphere, Eigen::Affine3d::Identity());
        points = dCollide(geom_a, sphere, 1,  &cg[0], sizeof cg[0]);
        BOOST_CHECK(points == 0);
        points = dCollide(geom_a, sphere, 1,  &cg[0], sizeof cg[0]);
        BOOST_CHECK(points == 0);

        // call interface with wrong user data type
        envire::core::Item<Eigen::Vector3d>::Ptr user_data_c(new envire::core::Item<Eigen::Vector3d>);
        BOOST_CHECK_THROW(c->createNewCollisionObject(user_data_c), DownCastException<Eigen::AlignedBox3d>);

        // call interface with wrong geom type
        BOOST_CHECK_THROW(c->getUserData(sphere), WrongGeomClassID);

        dGeomDestroy(geom_a);
        dGeomDestroy(geom_b);
        dGeomDestroy(sphere);
    }
    dCloseODE();
}