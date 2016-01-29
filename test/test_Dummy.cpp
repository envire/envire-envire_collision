#include <boost/test/unit_test.hpp>
#include <envire_collision/Dummy.hpp>

using namespace envire_collision;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    envire_collision::DummyClass dummy;
    dummy.welcome();
}
