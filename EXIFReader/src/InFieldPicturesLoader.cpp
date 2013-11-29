// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-04-04

//#include <iostream>
#include <algorithm>

#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp> 

#include <nifti_pics_server_util/AnnotatedPicUtil.h>
#include <nifti_pics_server_util/EXIFInfo.h>

#include "AnnotatedPicsManager.h"
#include "EXIFParser.h"
#include "FileReadingUtil.h"

#include"InFieldPicturesLoader.h"

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

                InFieldPicturesLoader::InFieldPicturesLoader(const boost::filesystem::path &folder, AnnotatedPicsManager* picsMgr, const EXIFReader_msgs::GeoCoordinate& mapOriginLatitude, const EXIFReader_msgs::GeoCoordinate& mapOriginLongitude, const double& mapOriginDirection)
                : PicturesLoader(folder, picsMgr, mapOriginLatitude, mapOriginLongitude, mapOriginDirection)
                {

                }

                void InFieldPicturesLoader::scanFolderForNewPictures()
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

                        // 2) Parses the EXIF info
                        EXIFInfo info;
                        errorCode = EXIFParser::parseEXIF(buffer, fileSize, info);

                        if (errorCode != 0)
                        {
                            picturesToIgnore.push_back(path);
                            cerr << "Cannot parse EXIF info. Error [" << errorCode << "]." << endl;
                            continue;
                        }

                        // 3) Copy to the ROS structure
                        AnnotatedPicture* picture = new AnnotatedPicture();
                        picture->filename = path.filename().string();
                        if (info.dateTimeOriginal != NULL)
                        {
                            //picture->dateTime = convertToROS(info.dateTimeOriginal); // To do: make that work
                            picture->dateTimeFormatted = info.dateTimeOriginal;
                        }
                        picture->fieldOfViewHorizontal = info.fieldOfViewHorizontal;
                        picture->fieldOfViewVertical = info.fieldOfViewVertical;
                        picture->latitude.degrees = info.gpsLatitude.degrees;
                        picture->latitude.minutes = info.gpsLatitude.minutes;
                        picture->latitude.seconds = info.gpsLatitude.seconds;
                        picture->longitude.degrees = info.gpsLongitude.degrees;
                        picture->longitude.minutes = info.gpsLongitude.minutes;
                        picture->longitude.seconds = info.gpsLongitude.seconds;
                        if (info.gpsImgDirection != 0)
                            picture->direction = info.gpsImgDirection;

                        // If there are no GPS coordinates, then just leave X,Y at 0,0
                        if (picture->latitude.degrees != 0)
                        {
                            //ROS_INFO_STREAM("mapOrigin latitude longitude: " << mapOriginLatitude.degrees + mapOriginLatitude.minutes / 60 + mapOriginLatitude.seconds / 3600 << ", " << mapOriginLongitude.degrees + mapOriginLongitude.minutes / 60 + mapOriginLongitude.seconds / 3600);
                            //ROS_INFO_STREAM("picture latitude longitude: " << picture->latitude.degrees + picture->latitude.minutes / 60 + picture->latitude.seconds / 3600 << ", " << picture->longitude.degrees + picture->longitude.minutes / 60 + picture->longitude.seconds / 3600);

                            const geodesy::UTMPoint utmPoint = AnnotatedPicUtil::getUTMPoint(picture);

                            const geodesy::UTMPoint mapOrigin(AnnotatedPicUtil::getGeoPoint(mapOriginLatitude, mapOriginLongitude));

                            //ROS_INFO_STREAM("mapOrigin: " << mapOrigin.easting << ", " << mapOrigin.northing);
                            //ROS_INFO_STREAM("pictureToBeSent: " << utmPoint.easting << ", " << utmPoint.northing);

                            double x_diff = utmPoint.easting - mapOrigin.easting;
                            double y_diff = utmPoint.northing - mapOrigin.northing;

                            //ROS_INFO_STREAM("east _diff: " << x_diff << " north _diff:" << y_diff);

                            picture->pose.position.x = -x_diff * cos(mapOriginDirection) - y_diff * sin(mapOriginDirection);
                            picture->pose.position.y = x_diff * sin(mapOriginDirection) - y_diff * cos(mapOriginDirection);

                            //ROS_INFO_STREAM("On /map frame [new]: " << picture->pose.position.x << ", " << picture->pose.position.y);
                        }

                        picture->sourceType = AnnotatedPicture::SOURCE_TYPE_EXTERNAL;
                        picture->source = "Mobile Device"; // Todo: set with a more meaningful value. (Try to read the device's serial number from the EXIF Data)

                        // 4) Copies the entire file in the data portion of the ROS message
                        picture->completeFile.resize(fileSize);
                        memcpy(picture->completeFile.data(), buffer, fileSize);

                        // 5) Parses the annotation text file
                        std::stringstream pathTextFile;
                        pathTextFile << path.parent_path().string() << "/" << path.stem().string() << ".txt";                       
                        loadAnnotationFromFile(pathTextFile.str(), picture->annotation);
                        
                        // 6) Updates the internal lists of pictures

                        picsMgr->addPicture(picture);

                        // 7) For debugging: displays the EXIF info of this newly added picture

                        AnnotatedPicUtil::printOutAnnotatedPictureDetails(picture);
                    }

                }

                void InFieldPicturesLoader::modifyPictureAnnotationOnDrive(const std::string& filename, const std::string& annotation)
                {
                    // 1) Removes the file extension and replaces it with .txt

                    // I do not use the boost function .stem() because it adds double-quotes in the middle of the path later on when I compose it
                    string stem = filename.substr(0, filename.find_last_of("."));
                    stem.append(".txt");
                    fs::path path = folder / stem;                  

                    // 2) Write the new annotation
                    
                    fs::ofstream file(path);
                    file << annotation;
                    file.close();
                    
                    
                }
                
                
                ////////////
                // STATIC //
                ////////////                
                
                void InFieldPicturesLoader::loadAnnotationFromFile(const boost::filesystem::path& path, std::string& annotation)
                {
                    // Checks if the file exists

                    if (is_regular_file(path) == false)
                    {
                        annotation = "";
                        return;
                    }

                    // Loads the entire text file in one shot

                    std::ifstream ifs(path.string().c_str()); 
                    std::stringstream buffer;
                    buffer << ifs.rdbuf();
                    
                    annotation = buffer.str();                    
                }
                

            }

        }
    }
}
