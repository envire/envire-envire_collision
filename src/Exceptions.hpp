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

#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <envire_core/items/ItemBase.hpp>
#include <envire_core/util/Demangle.hpp>

namespace envire { namespace collision
{

    class WrongGeomClassID : public std::exception
    {
    public:
        explicit WrongGeomClassID(int expected_id, int geom_id) :
            msg("Can't handle geom object with class ID " + boost::lexical_cast<std::string>(geom_id) + ", expected ID is " + boost::lexical_cast<std::string>(expected_id) + "!") {}
        virtual char const * what() const throw() { return msg.c_str(); }
        const std::string msg;
    };

    class CollisionPluginNotLinkedException : public std::exception
    {
    public:
        explicit CollisionPluginNotLinkedException(const envire::core::ItemBase::Ptr user_data) :
            msg("Collision plugin library of class " + user_data->getClassName() + " not linked! Can't create a collision object.") {}
        virtual char const * what() const throw() { return msg.c_str(); }
        const std::string msg;
    };

    template<class T>
    class DownCastException : public std::exception
    {
    public:
        explicit DownCastException(const envire::core::ItemBase::Ptr user_data) :
            msg("Failed to downcast " + user_data->getClassName() + " to type " + envire::core::demangleTypeName(std::type_index(typeid(T))) + "!") {}
        virtual char const * what() const throw() { return msg.c_str(); }
        const std::string msg;
    };

}}