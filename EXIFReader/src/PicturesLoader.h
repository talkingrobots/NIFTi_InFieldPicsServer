// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-04-04

#ifndef EU_NIFTI_MISC_EXIF_READER_PICTURES_LOADER_H
#define EU_NIFTI_MISC_EXIF_READER_PICTURES_LOADER_H

#include <list>
#include <map>
#include <string>

#include <boost/filesystem.hpp>

#include "EXIFReader_msgs/AnnotatedPicture.h"

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                class AnnotatedPicsManager;
                
                /**
                 * Reads in picture files and parses the metadata 
                 * @param folder
                 */
                class PicturesLoader
                {
                public:
                    
                    PicturesLoader(const boost::filesystem::path& folder, AnnotatedPicsManager* picsMgr, const EXIFReader_msgs::GeoCoordinate& mapOriginLatitude, const EXIFReader_msgs::GeoCoordinate& mapOriginLongitude, const double& mapOriginDirection);
                    
                    virtual void scanFolderForNewPictures() = 0;
                    
                protected:
                    
                    void discoverNewPicturePaths(std::list<boost::filesystem::path> &newPicturePaths) const;
                    
                    // Pictures that are problematic and that should be ignored in subsequent scans
                    std::list<boost::filesystem::path> picturesToIgnore;
                                       
                    const boost::filesystem::path folder;

                    AnnotatedPicsManager* picsMgr;
                    
                    const EXIFReader_msgs::GeoCoordinate& mapOriginLatitude;
                    const EXIFReader_msgs::GeoCoordinate& mapOriginLongitude;
                    const double& mapOriginDirection;
                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_PICTURES_LOADER_H
