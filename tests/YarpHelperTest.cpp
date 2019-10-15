#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include <WalkingControllers/YarpHelper/Helper.h>

TEST_CASE("Check GetNumberFromSearchable", "[GetNumberFromSearchable]")
{
    yarp::os::Bottle bottle;

    yarp::os::Property& dictionary = bottle.addDict();
    dictionary.put("number", 1);

    int number;
    WalkingControllers::YarpHelper::getNumberFromSearchable(dictionary, "number", number);

    REQUIRE(number == 1);
}
