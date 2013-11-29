// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-04-04

#include <algorithm>
#include <string>

#include <boost/foreach.hpp> 
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>

#include <nifti_pics_server_util/AnnotatedPicUtil.h>
#include <nifti_pics_server_util/EXIFInfo.h>

#include "AnnotatedPicsManager.h"
#include "EXIFParser.h"
#include "FileReadingUtil.h"

#include "RobotPicturesLoader.h"

using namespace std;
using namespace EXIFReader_msgs;
using namespace eu::nifti::misc::nifti_pics_server_util;
namespace fs = boost::filesystem;

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                const double PI = 3.141592654;

                // The number of lines in the metadata file that must be there. After that may come an annotation.
                const u_int METADATA_MIN_LENGTH = 11;

                RobotPicturesLoader::RobotPicturesLoader(const boost::filesystem::path &folder, AnnotatedPicsManager* picsMgr, const EXIFReader_msgs::GeoCoordinate& mapOriginLatitude, const EXIFReader_msgs::GeoCoordinate& mapOriginLongitude, const double& mapOriginDirection)
                : PicturesLoader(folder, picsMgr, mapOriginLatitude, mapOriginLongitude, mapOriginDirection)
                {

                }

                void RobotPicturesLoader::scanFolderForNewPictures()
                {
                    std::list<fs::path> newPicturePaths;

                    discoverNewPicturePaths(newPicturePaths); // newPicturePaths are sorted (most recent to oldest)

                    BOOST_FOREACH(fs::path const &path, newPicturePaths)
                    {

                        int errorCode = 0;

                        // 1) Loads the file
                        unsigned char *buffer = NULL;
                        unsigned long fileSize;

                        cout << "Reading picture " << path.filename() << endl;

                        errorCode = FileReadingUtil::loadFile(path.string().c_str(), buffer, fileSize);

                        if (errorCode != 0)
                        {
                            picturesToIgnore.push_back(path);
                            cerr << "Cannot read picture file: " << errorCode << endl;
                            continue;
                        }

                        // 2) Parses the metadata
                        std::stringstream pathTextFile;
                        pathTextFile << path.parent_path().string() << "/" << path.stem().string() << ".txt";


                        geometry_msgs::Pose pose;
                        ros::Time dateTime;
                        std::string dateTimeFormatted;
                        std::string annotation;
                        double fieldOfViewHorizontal, fieldOfViewVertical;
                        double orientation;

                        std::vector<std::string> metadata;
                        try
                        {
                            loadMetadataFromFile(pathTextFile.str(), metadata);
                        }
                        catch (const std::string& ex)
                        {
                            cerr << "Problem with the metadata file for picture [" << path.filename() << "]: " << ex << endl;
                            picturesToIgnore.push_back(path);
                            continue;
                        }
                        convertMetadata(metadata, pose, orientation, dateTime, fieldOfViewHorizontal, fieldOfViewVertical, annotation);


                        // 3) Copy to the AnnotatedPicture ROS structure
                        AnnotatedPicture* picture = new AnnotatedPicture();
                        picture->filename = path.filename().string();
                        picture->dateTime = dateTime;
                        //picture->dateTimeFormatted = dateTimeFormatted; // Todo: Format this? Or let the user do it?

                        picture->sourceType = AnnotatedPicture::SOURCE_TYPE_OCU;
                        picture->source = "OCU"; // Todo: set with a more meaningful value.

                        picture->annotation = annotation;

                        picture->pose = pose;

                        // Calculates the GPS coordinate from the position w.r.t. /map
                        {
                            //ROS_INFO_STREAM("mapOrigin latitude longitude: " << mapOriginLatitude.degrees + mapOriginLatitude.minutes / 60 + mapOriginLatitude.seconds / 3600 << ", " << mapOriginLongitude.degrees + mapOriginLongitude.minutes / 60 + mapOriginLongitude.seconds / 3600);
                            //ROS_INFO_STREAM("On /map frame [new]: " << picture->pose.position.x << ", " << picture->pose.position.y);

                            const geodesy::UTMPoint mapOrigin(AnnotatedPicUtil::getGeoPoint(mapOriginLatitude, mapOriginLongitude));

                            //ROS_INFO_STREAM("mapOrigin: " << mapOrigin.easting << ", " << mapOrigin.northing);

                            double pictureEasting = mapOrigin.easting + picture->pose.position.x * cos(mapOriginDirection) + picture->pose.position.y * sin(mapOriginDirection);
                            double pictureNorthing = mapOrigin.northing - picture->pose.position.x * sin(mapOriginDirection) + picture->pose.position.y * cos(mapOriginDirection);

                            //ROS_INFO_STREAM("pictureEasting: " << pictureEasting << " pictureNorthing:" << pictureNorthing);

                            const geodesy::UTMPoint pictureUTMPoint(pictureEasting, pictureNorthing, mapOrigin.zone, mapOrigin.band); // I'm assuming that we stay in the same zone and band, because we take pictures close to the map origin
                            geographic_msgs::GeoPoint pictureGeoPoint = geodesy::toMsg(pictureUTMPoint);
                            picture->latitude.degrees = pictureGeoPoint.latitude;
                            picture->longitude.degrees = pictureGeoPoint.longitude;

                            //ROS_INFO_STREAM("pictureGeoPoint: " << pictureGeoPoint.latitude << ", " << pictureGeoPoint.longitude);
                        }

                        // Calculates the GPS direction from the orientation w.r.t. /map
                        {
                            const double orientationDegreesClockwiseFromMapUp = -(orientation * 180 / PI - 90);
                            const double orientationDegreesClockwiseFromNorth = orientationDegreesClockwiseFromMapUp + mapOriginDirection;
                            picture->direction = fmod(orientationDegreesClockwiseFromNorth, 360); // Converts the value from (wrt /map) to (wrt North). North is 0, increasing clockwise
                            //ROS_INFO_STREAM("Orientation (/map): " << orientation << " Direction (GPS): " << picture->direction);
                        }

                        picture->fieldOfViewHorizontal = fieldOfViewHorizontal;
                        picture->fieldOfViewVertical = fieldOfViewVertical;

                        // 4) Copies the entire file in the data portion of the ROS message
                        picture->completeFile.resize(fileSize);
                        memcpy(picture->completeFile.data(), buffer, fileSize);

                        // 5) Updates the internal lists of pictures

                        picsMgr->addPicture(picture);

                        // 6) For debugging: displays the EXIF info of this newly added picture

                        AnnotatedPicUtil::printOutAnnotatedPictureDetails(picture);
                    }
                }

                void RobotPicturesLoader::modifyPictureMetadataOnDrive(const std::string& filename, const std::string& annotation)
                {
                    //ROS_INFO_STREAM("IN modifyPictureMetadataOnDrive: " << filename);

                    // 1) Read the metadata from file

                    std::vector<std::string> metadata;
                    
                    // Removes the extension and replaces it with .txt
                    // I do not use the boost function .stem() because it adds double-quotes in the middle of the path later on when I compose it
                    string stem = filename.substr(0, filename.find_last_of("."));
                    stem.append(".txt");
                    fs::path path = folder / stem;                  

                    try
                    {
                        loadMetadataFromFile(path, metadata);
                    }
                    catch (const std::string& ex)
                    {
                        cerr << "Problem with the metadata file for picture [" << path.filename() << "]: " << ex << endl;
                        return;
                    }


                    // 2) Open the file for output

                    fs::ofstream file(path);

                    // 3) Write the lines representing the old metadata up to the annotation part
                    
                    for (u_int i = 0; i < METADATA_MIN_LENGTH + 2; i++)
                    {
                        file << metadata.at(i);
                        file << std::endl;
                    }
                    
                    // 4) Write the new annotation
                    
                    file << annotation;
                    
                    file.close();

                }


                ////////////
                // STATIC //
                ////////////

                void RobotPicturesLoader::loadMetadataFromFile(const boost::filesystem::path& path, std::vector<std::string>& metadata)
                {
                    // 1) Checks if the file exists

                    if (is_regular_file(path) == false)
                    {
                        std::stringstream ss;
                        ss << "This metadata file does not exist: " << path;
                        throw ss.str();
                    }


                    // 1) Load the text file, line by line

                    fs::ifstream fileStream(path);
                    while (fileStream)
                    {
                        std::string buffer;
                        std::getline(fileStream, buffer);
                        if (fileStream)
                        {
                            //std::cout << "Read in: " << buffer << "\n";
                            metadata.push_back(buffer);
                        }
                    }

                    // 2) Check for completeness

                    if (metadata.size() < METADATA_MIN_LENGTH)
                    {
                        std::stringstream ss;
                        ss << "Error in the metadata: it does not contain the required " << METADATA_MIN_LENGTH << " fields. It contains " << metadata.size();
                        throw ss.str();
                    }
                }

                void RobotPicturesLoader::convertMetadata(const std::vector<std::string>& metadata, geometry_msgs::Pose& pose, double& orientation, ros::Time& time, double& fieldOfViewHorizontal, double& fieldOfViewVertical, std::string & annotation)
                {
                    // Todo: check that the metadata is valid, because at this point it is assumed to be

                    time = ros::Time(atof(metadata.at(0).c_str()));
                    pose.position.x = atof(metadata.at(1).c_str());
                    pose.position.y = atof(metadata.at(2).c_str());
                    pose.position.z = atof(metadata.at(3).c_str());
                    pose.orientation.w = atof(metadata.at(4).c_str());
                    pose.orientation.x = atof(metadata.at(5).c_str());
                    pose.orientation.y = atof(metadata.at(6).c_str());
                    pose.orientation.z = atof(metadata.at(7).c_str());
                    orientation = atof(metadata.at(8).c_str());
                    fieldOfViewHorizontal = atof(metadata.at(9).c_str());
                    fieldOfViewVertical = atof(metadata.at(10).c_str());


                    std::stringstream ss;
                    for (u_int i = METADATA_MIN_LENGTH + 2; i < metadata.size(); i++)
                    {
                        if (i != METADATA_MIN_LENGTH + 2) // Adds an endl at all lines except after the last one
                        {
                            ss << std::endl;
                        }

                        ss << metadata.at(i);
                    }
                    annotation = ss.str();
                }


            } // End class
        }
    }
}
