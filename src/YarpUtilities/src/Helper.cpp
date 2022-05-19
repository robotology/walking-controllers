/**
 * @file Helper.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>

#include <WalkingControllers/YarpUtilities/Helper.h>

using namespace WalkingControllers;

bool YarpUtilities::addVectorOfStringToProperty(yarp::os::Property& prop, const std::string& key,
                                             const std::vector<std::string>& list)
{
    // check if the key already exists
    if(prop.check(key))
    {
        yError() << "[addVectorOfStringToProperty] The property already exist.";
        return false;
    }

    prop.addGroup(key);
    yarp::os::Bottle& bot = prop.findGroup(key).addList();
    for(size_t i=0; i < list.size(); i++)
        bot.addString(list[i].c_str());

    return true;
}

bool YarpUtilities::yarpListToStringVector(yarp::os::Value*& input, std::vector<std::string>& output)
{
    // clear the std::vector
    output.clear();

    // check if the yarp value is a list
    if(!input->isList())
    {
        yError() << "[yarpListToStringVector] The input is not a list.";
        return false;
    }

    yarp::os::Bottle *bottle = input->asList();
    for(int i = 0; i < bottle->size(); i++)
    {
        // check if the elements of the bottle are strings
        if(!bottle->get(i).isString())
        {
            yError() << "[yarpListToStringVector] There is a field that is not a string.";
            return false;
        }
        output.push_back(bottle->get(i).asString());
    }
    return true;
}

bool YarpUtilities::getStringFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                         std::string& string)
{
    yarp::os::Value* value;
    if(!config.check(key, value))
    {
        yError() << "[getStringFromSearchable] Missing field "<< key;
        return false;
    }

    if(!value->isString())
    {
        yError() << "[getStringFromSearchable] the value is not a string.";
        return false;
    }

    string = value->asString();
    return true;
}

bool YarpUtilities::yarpListToBoolVector(yarp::os::Value*& input, std::vector<bool>& output)
{
    // clear the std::vector
    output.clear();

    // check if the yarp value is a list
    if(!input->isList())
    {
        yError() << "[yarpListToBoolVector] The input is not a list.";
        return false;
    }

    yarp::os::Bottle *bottle = input->asList();
    for(int i = 0; i < bottle->size(); i++)
    {
        // check if the elements of the bottle are bool or integer
        if(!bottle->get(i).isBool() && !bottle->get(i).isInt32())
        {
            yError() << "[yarpListToBoolVector] There is a field that is not a bool.";
            return false;
        }
        output.push_back(bottle->get(i).asBool());
    }
    return true;
}

bool YarpUtilities::getNumberFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                         double& number)
{
    yarp::os::Value* value;
    if(!config.check(key, value))
    {
        yError() << "[getNumberFromSearchable] Missing field "<< key;
        return false;
    }

    if(!value->isFloat64())
    {
        yError() << "[getNumberFromSearchable] the value is not a double.";
        return false;
    }

    number = value->asFloat64();
    return true;
}

bool YarpUtilities::getNumberFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                         int& number)
{
    yarp::os::Value* value;
    if(!config.check(key, value))
    {
        yError() << "[getNumberFromSearchable] Missing field "<< key;
        return false;
    }

    if(!value->isInt32())
    {
        yError() << "[getNumberFromSearchable] the value is not an integer.";
        return false;
    }

    number = value->asInt32();
    return true;
}

void YarpUtilities::populateBottleWithStrings(yarp::os::Bottle& bottle, const std::initializer_list<std::string>& strings)
{
    for(const auto& string : strings)
        bottle.addString(string);
}

bool YarpUtilities::getVectorOfBooleanFromSearchable(const yarp::os::Searchable& config,const std::string& key,std::vector <bool> & output)
{
    yarp::os::Value* valuePtr;
    if(!config.check(key, valuePtr))
    {
        yError() << "[getiDynTreeVectorFixSizeFromSearchable] Missing field "<< key;
        return false;
    }

    yarp::os::Value value=*valuePtr;
    yarp::os::Bottle *inputPtr = value.asList();

    if (inputPtr->size() != output.size())
    {
        yError() << "[getVectorOfBooleanFromSearchable] The dimension set in the configuration file is not "
                 << output.size();
        return false;
    }

    for (unsigned int i = 0; i < inputPtr->size(); i++)
    {
        if (!inputPtr->get(i).isBool())
        {
            yError() << "[getVectorOfBooleanFromSearchable] The input is expected to be boolean";
            return false;
        }
        output[i]= inputPtr->get(i).asBool();
    }

    return true;
}
