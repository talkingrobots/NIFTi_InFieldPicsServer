// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-04-15

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/io.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include"CVPicUtil.h"


namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                bool CVPicUtil::saveImage(const sensor_msgs::ImageConstPtr& imageMsg, const std::string& filename)
                {
                    cv::Mat imageCV;
                    loadCVImage(imageMsg, imageCV);

                    try
                    {
                        cv::imwrite(filename, imageCV);
                    }
                    catch (cv_bridge::Exception& ex)
                    {
                        ROS_ERROR_STREAM("Problem while trying to save the image file [" << filename << "]: " << ex.what());
                        return false;
                    }

                    return true;
                }

                bool CVPicUtil::loadCVImage(const sensor_msgs::ImageConstPtr& source, cv::Mat& destination)
                {
                    cv::Mat imageCV;
                    try
                    {
                        imageCV = cv_bridge::toCvShare(source, source->encoding)->image;
                    }
                    catch (cv_bridge::Exception& ex)
                    {
                        ROS_ERROR("Unable to convert %s image to bgr8", source->encoding.c_str());
                        return false;
                    }
                    if (imageCV.empty())
                    {
                        ROS_ERROR("Couldn't save image, no data!");
                        return false;
                    }

                    // This block corrects a problem is the conversion function toCvShare
                    if (source->encoding == sensor_msgs::image_encodings::RGB8)
                    {
                        cv::cvtColor(imageCV, destination, CV_RGB2BGR);
                    }
                    else
                    {
                        destination = imageCV;
                    }
                    
                    // Benoit: I don't think that I need to deal with that... for now                    
//                    else if (source->encoding == sensor_msgs::image_encodings::RGBA8)
//                    {
//                        format = Ogre::PF_BYTE_RGBA;
//                    }

                    return true;
                }

            }

        }
    }
}
