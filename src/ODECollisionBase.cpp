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