// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-04-04

#ifndef EU_NIFTI_MISC_EXIF_READER_ROBOT_PICTURES_LOADER_H
#define EU_NIFTI_MISC_EXIF_READER_ROBOT_PICTURES_LOADER_H

#include "PicturesLoader.h"

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                /**
                 * Reads in the picture files from the OCU and parses the metadata
                 * @param folder
                 */
                class RobotPicturesLoader: public PicturesLoader
                {
                public:
                    
                    RobotPicturesLoader(const boost::filesystem::path &folder, AnnotatedPicsManager* picsMgr, const EXIFReader_msgs::GeoCoordinate& mapOriginLatitude, const EXIFReader_msgs::GeoCoordinate& mapOriginLongitude, const double& mapOriginDirection);
                    
                    void scanFolderForNewPictures();
                    
                    void modifyPictureMetadataOnDrive(const std::string& filename, const std::string& annotation);
                    
                protected:
                    
                    static void loadMetadataFromFile(const boost::filesystem::path& path, std::vector<std::string>& metadata);
                    static void convertMetadata(const std::vector<std::string>& metadata, geometry_msgs::Pose& pose, double& orientation, ros::Time& time, double& fieldOfViewHorizontal, double& fieldOfViewVertical, std::string & annotation);
                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_ROBOT_PICTURES_LOADER_H
