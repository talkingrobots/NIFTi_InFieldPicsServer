// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-02-18

#ifndef EU_NIFTI_MISC_EXIF_READER_ANNOTATED_PIC_UTIL_H
#define EU_NIFTI_MISC_EXIF_READER_ANNOTATED_PIC_UTIL_H

#include <list>

#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>

#include "EXIFReader_msgs/AnnotatedPicture.h"

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace nifti_pics_server_util
            {

                typedef std::map< const std::string, const EXIFReader_msgs::AnnotatedPicture*> MapOfAnnotatedPictures;
                
                /**
                 * Utilities for annotated pictures (contain EXIF data with GPS info)
                 * @param folder
                 */
                class AnnotatedPicUtil
                {
                public:
                    
                    static void printOutAnnotatedPictureDetails(const std::list< EXIFReader_msgs::AnnotatedPicture*> &pictures);
                    static void printOutAnnotatedPictureDetails(const EXIFReader_msgs::AnnotatedPicture* picture);
                    
                    static EXIFReader_msgs::AnnotatedPicture* cloneAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* source, bool copyCompleteFile = true);
                    static void cloneAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* source, EXIFReader_msgs::AnnotatedPicture* destination, bool copyCompleteFile = true);

                    static inline geographic_msgs::GeoPoint getGeoPoint(const EXIFReader_msgs::GeoCoordinate& latitude, const EXIFReader_msgs::GeoCoordinate& longitude)
                    {
                        geographic_msgs::GeoPoint geoPoint;
                        geoPoint.latitude = latitude.degrees + latitude.minutes / 60 + latitude.seconds / 3600;
                        geoPoint.longitude = longitude.degrees + longitude.minutes / 60 + longitude.seconds / 3600;

                        return geoPoint;
                    }

                    static inline geographic_msgs::GeoPoint getGeoPoint(const EXIFReader_msgs::AnnotatedPicture* picture)
                    {
                        return getGeoPoint(picture->latitude, picture->longitude);
                    }

                    static inline geodesy::UTMPoint getUTMPoint(const EXIFReader_msgs::AnnotatedPicture* picture)
                    {
                        return geodesy::UTMPoint(getGeoPoint(picture));
                    }
                    
                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_ANNOTATED_PIC_UTIL_H
