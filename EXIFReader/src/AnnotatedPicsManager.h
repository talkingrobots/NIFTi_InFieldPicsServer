// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2012-12-21

#ifndef EU_NIFTI_MISC_EXIF_READER_ANNOTATED_PICS_MGR_H
#define EU_NIFTI_MISC_EXIF_READER_ANNOTATED_PICS_MGR_H

#include <list>
#include <map>
#include <string>

#include <boost/filesystem.hpp>

#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>

#include <EXIFReader_msgs/AnnotatedPicture.h>

#include"InFieldPicturesLoader.h"
#include"RobotPicturesLoader.h"

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                typedef std::map< const std::string, const EXIFReader_msgs::AnnotatedPicture*> MapOfConstAnnotatedPictures;
                
                /**
                 * Manages pictures in a folder. Reads in the files and parses the EXIF data.
                 * @param folder
                 */
                class AnnotatedPicsManager
                {
                public:
                    
                    AnnotatedPicsManager(const boost::filesystem::path &folder, const EXIFReader_msgs::GeoCoordinate& mapOriginLatitude, const EXIFReader_msgs::GeoCoordinate& mapOriginLongitude, const double& mapOriginDirection);
                    ~AnnotatedPicsManager();
                                        
                    const EXIFReader_msgs::AnnotatedPicture* getPicture(const std::string& filename) const;
                    
                    const MapOfConstAnnotatedPictures& getPictures() const;
                    
                    void scanFoldersForNewPictures();
                    
                    const std::list< const EXIFReader_msgs::AnnotatedPicture*>& getNewPictures() const;
                    
                    void addPicture(EXIFReader_msgs::AnnotatedPicture* picture);
                    void modifyPicture(const std::string& filename, const std::string& annotation);
                    
                private:

                    MapOfConstAnnotatedPictures pictures;
                    std::list< const EXIFReader_msgs::AnnotatedPicture*> newPictures;
                    
                    const EXIFReader_msgs::GeoCoordinate mapOriginLatitude;
                    const EXIFReader_msgs::GeoCoordinate mapOriginLongitude;
                    const double mapOriginDirection;
                    
                    InFieldPicturesLoader inFieldPicturesLoader;
                    RobotPicturesLoader robotPicturesLoader;
                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_ANNOTATED_PICS_MGR_H
