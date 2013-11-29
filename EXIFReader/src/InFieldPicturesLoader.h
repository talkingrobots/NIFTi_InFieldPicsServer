// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-04-04

#ifndef EU_NIFTI_MISC_EXIF_READER_IN_FIELD_PICTURES_LOADER_H
#define EU_NIFTI_MISC_EXIF_READER_IN_FIELD_PICTURES_LOADER_H

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
                 * Reads in the files and parses the EXIF data for photos from the in-field rescuer
                 * @param folder
                 */
                class InFieldPicturesLoader: public PicturesLoader
                {
                public:
                    
                    InFieldPicturesLoader(const boost::filesystem::path &folder, AnnotatedPicsManager* picsMgr, const EXIFReader_msgs::GeoCoordinate& mapOriginLatitude, const EXIFReader_msgs::GeoCoordinate& mapOriginLongitude, const double& mapOriginDirection);
                    
                    void scanFolderForNewPictures();
                    
                    void modifyPictureAnnotationOnDrive(const std::string& filename, const std::string& annotation);
                
                protected:
                    static void loadAnnotationFromFile(const boost::filesystem::path& path, std::string& annotation);
                    
                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_IN_FIELD_PICTURES_LOADER_H
