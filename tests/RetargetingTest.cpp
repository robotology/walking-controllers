#include <WalkingControllers/RetargetingHelper/Helper.h>
#include <iostream>

int main()
{
    yarp::os::Bottle bottle;

    yarp::os::Property& dictionary = bottle.addDict();
    dictionary.put("number", 1);

    RetargetingClient client;
    client.initialize(bottle, "test", 1);

    return 0;
}
