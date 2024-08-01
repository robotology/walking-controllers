#define CATCH_CONFIG_MAIN
#include <WalkingControllers/YarpUtilities/Helper.h>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Check GetNumberFromSearchable", "[GetNumberFromSearchable]") {
  yarp::os::Bottle bottle;

  yarp::os::Property &dictionary = bottle.addDict();
  dictionary.put("number", 1);

  int number;
  WalkingControllers::YarpUtilities::getNumberFromSearchable(dictionary,
                                                             "number", number);

  REQUIRE(number == 1);
}
