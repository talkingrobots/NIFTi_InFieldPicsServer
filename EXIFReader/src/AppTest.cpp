// rosrun EXIFReader EXIFReader "/home/robot/Pictures/Party_Geo.JPG"

//#include <ros/ros.h>
//#include <std_msgs/String.h>

//#include <iostream>

#include "EXIFParser.h"
#include "EXIFInfo.h"
#include "FileReadingUtil.h"

using namespace eu::nifti::misc::EXIFReader;

void displayEXIFInfo(EXIFInfo& exifInfo)
{
    printf("\n\nRESULT:\n******\n\n");

    // Dump whatever information is available
    if (exifInfo.cameraModel)
        printf("Camera model      : %s\n", exifInfo.cameraModel);
    if (exifInfo.cameraMake)
        printf("Camera make       : %s\n", exifInfo.cameraMake);
    if (exifInfo.focalLength)
        printf("Lens focal length : %umm\n", exifInfo.focalLength);
    if (exifInfo.FStop)
        printf("Lens F-number     : f/%g\n", exifInfo.FStop);
    if (exifInfo.exposureTime)
        printf("Exposure          : 1/%gs\n", 1.0 / exifInfo.exposureTime);
    if (exifInfo.imgDescription)
        printf("Image description : %s\n", exifInfo.imgDescription);
    if (exifInfo.dateTimeModified)
        printf("Date/time modified: %s\n", exifInfo.dateTimeModified);
    if (exifInfo.dateTimeOriginal)
        printf("Date/time original: %s\n", exifInfo.dateTimeOriginal);

    if (exifInfo.gpsLatitude.degrees)
        printf("gpsLatitude: %f %f %f\n", exifInfo.gpsLatitude.degrees, exifInfo.gpsLatitude.minutes, exifInfo.gpsLatitude.seconds);
    if (exifInfo.gpsLongitude.degrees)
        printf("gpsLongitude: %f %f %f\n", exifInfo.gpsLongitude.degrees, exifInfo.gpsLongitude.minutes, exifInfo.gpsLongitude.seconds);

}


int main(int argc, char** argv)
{
    // Usage: // rosrun EXIFReader EXIFReader "/home/robot/Pictures/Party_Geo.JPG"
    
    //ROS_INFO("Application started");

    int errorCode = 0;

    // 1) Loads the file
    unsigned char *buffer = NULL;
    unsigned long fileSize;  

    errorCode = FileReadingUtil::loadFile(argv[1], buffer, fileSize);

    if (errorCode != 0)
    {
        return errorCode;
    }

    

    // 2) Parses the EXIF info
    
    EXIFInfo result;
    errorCode = EXIFParser::parseEXIF(buffer, fileSize, result);

    if (errorCode != 0)
    {
        printf("Could not parse the EXIF Info. Error %i\n", errorCode);
    }
    else
    {
        displayEXIFInfo(result);
    }

    // 3) Cleans up
    delete[] buffer;
    
    return errorCode;

}