// Benoit 2013-01-11

#include <string>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <EXIFReader_msgs/GeoCoordinate.h>

#include "InFieldPicsServer.h"

using namespace eu::nifti::misc::EXIFReader;
using namespace EXIFReader_msgs;

const double PI = 3.14159265358979323846264338327950288419716939937510;

GeoCoordinate getGeoCoordinate(double degrees, double minutes, double seconds)
{
    GeoCoordinate coord;
    coord.degrees = degrees;
    coord.minutes = minutes;
    coord.seconds = seconds;

    return coord;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "GeoPictureListener");

    ros::NodeHandle nodeHandle("~"); // Looks for the parameters in the private namespace

    std::string folder;
    if (!nodeHandle.getParam("folder", folder))
    {
        folder = "/home/robot/Pictures/";
    }

    double mapOriginLatitudeDegrees, mapOriginLatitudeMinutes, mapOriginLatitudeSeconds;
    double mapOriginLongitudeDegrees, mapOriginLongitudeMinutes, mapOriginLongitudeSeconds;
    double mapOriginDirection;

    nodeHandle.getParam("mapOriginLatitudeDegrees", mapOriginLatitudeDegrees);
    nodeHandle.getParam("mapOriginLatitudeMinutes", mapOriginLatitudeMinutes);
    nodeHandle.getParam("mapOriginLatitudeSeconds", mapOriginLatitudeSeconds);

    nodeHandle.getParam("mapOriginLongitudeDegrees", mapOriginLongitudeDegrees);
    nodeHandle.getParam("mapOriginLongitudeMinutes", mapOriginLongitudeMinutes);
    nodeHandle.getParam("mapOriginLongitudeSeconds", mapOriginLongitudeSeconds);

    nodeHandle.getParam("mapOriginDirection", mapOriginDirection);

    GeoCoordinate mapOriginLatitude = getGeoCoordinate(mapOriginLatitudeDegrees, mapOriginLatitudeMinutes, mapOriginLatitudeSeconds);
    GeoCoordinate mapOriginLongitude = getGeoCoordinate(mapOriginLongitudeDegrees, mapOriginLongitudeMinutes, mapOriginLongitudeSeconds);

    InFieldPicsServer server(nodeHandle, folder, mapOriginLatitude, mapOriginLongitude, mapOriginDirection*PI/180); // Converts the orientation to radians

    // For debugging, I wait a second to let the subscribers connect
    ros::Time::sleepUntil(ros::Time::now() + ros::Duration(1));

    server.run();

    return 0;

}