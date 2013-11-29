// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-01-10

#ifndef EU_NIFTI_MISC_EXIF_READER_IN_FIELD_PICS_SERVER_H
#define EU_NIFTI_MISC_EXIF_READER_IN_FIELD_PICS_SERVER_H

#include <list>
#include <string>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <image_transport/image_transport.h>

#include <EXIFReader_msgs/GeoCoordinate.h>
#include <EXIFReader_msgs/GetAnnotatedPicture.h>
#include <EXIFReader_msgs/GetAllAnnotatedPictures.h>
#include <EXIFReader_msgs/Modification.h>

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
                 * Manages the pictures coming from a mobile device and publishes them to ROS
                 * @param folder
                 * @param nodeHandle
                 */
                class InFieldPicsServer
                {
                public:

                    InFieldPicsServer(ros::NodeHandle &nodeHandle, const std::string &folder, const EXIFReader_msgs::GeoCoordinate& mapOriginLatitude, const EXIFReader_msgs::GeoCoordinate& mapOriginLongitude, const double& mapOriginDirection);

                    void run();

                private:
                    bool onGetAnnotatedPictureRequest(EXIFReader_msgs::GetAnnotatedPicture::Request &request, EXIFReader_msgs::GetAnnotatedPicture::Response &response);
                    bool onGetAllAnnotatedPicturesRequest(EXIFReader_msgs::GetAllAnnotatedPictures::Request &request, EXIFReader_msgs::GetAllAnnotatedPictures::Response &response);

                    void onNewSnapshot(const sensor_msgs::ImageConstPtr& imageMsg);
                    void onNewMetadata(const std_msgs::Header& metadataMsg);
                    void onModification(const EXIFReader_msgs::Modification& modificationMsg);
                    
                    std::string folder;
                    
                    // These two listen for snapshots coming from the OCUs
                    image_transport::ImageTransport imageTransport;
                    image_transport::Subscriber imageSubscriber;
                    ros::Subscriber subscriberMetadata;
                    ros::Subscriber subscriberModification;
                    
                    AnnotatedPicsManager *mgr;
                    ros::Publisher pubNew;
                    ros::ServiceServer servGetAnnotatedPicture;
                    ros::ServiceServer servGetAllAnnotatedPictures;
                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_IN_FIELD_PICS_SERVER_H
