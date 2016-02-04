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