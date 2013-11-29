// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-04-15

#ifndef EU_NIFTI_MISC_EXIF_READER_CV_PIC_UTIL_H
#define EU_NIFTI_MISC_EXIF_READER_CV_PIC_UTIL_H


namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {
                
                /**
                 * Utilities for OpenCV pictures
                 * @param folder
                 */
                class CVPicUtil
                {
                public:
                    
                    static bool saveImage(const sensor_msgs::ImageConstPtr& imageMsg, const std::string& filename);
                    static bool loadCVImage(const sensor_msgs::ImageConstPtr& source, cv::Mat& destination);
                    
                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_CV_PIC_UTIL_H
