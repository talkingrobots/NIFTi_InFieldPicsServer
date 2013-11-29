// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-01-10

#include <stdexcept>

#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/foreach.hpp>
#include <boost/filesystem/fstream.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <EXIFReader_msgs/AnnotatedPicture.h>

#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>

#include <nifti_pics_server_util/AnnotatedPicUtil.h>
#include <nifti_pics_server_util/ExceptionWithString.h>

#include "AnnotatedPicsManager.h"
#include "CVPicUtil.h"

#include"InFieldPicsServer.h"

using namespace std;
using namespace EXIFReader_msgs;

namespace fs = boost::filesystem;

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                const std::string TOPIC_MODIFY = "/inFieldPicsServer/modify";
                
                InFieldPicsServer::InFieldPicsServer(ros::NodeHandle &nodeHandle, const std::string &folder, const GeoCoordinate& mapOriginLatitude, const GeoCoordinate& mapOriginLongitude, const double& mapOriginDirection)
                : folder(folder)
                , imageTransport(nodeHandle)
                {
                    mgr = new AnnotatedPicsManager(boost::filesystem::path(folder), mapOriginLatitude, mapOriginLongitude, mapOriginDirection);

                    // ROS publishers and servers
                    pubNew = nodeHandle.advertise<AnnotatedPicture > ("/inFieldPicsServer/new", 100, false);
                    servGetAnnotatedPicture = nodeHandle.advertiseService("/inFieldPicsServer/get", &InFieldPicsServer::onGetAnnotatedPictureRequest, this);
                    servGetAllAnnotatedPictures = nodeHandle.advertiseService("/inFieldPicsServer/getAll", &InFieldPicsServer::onGetAllAnnotatedPicturesRequest, this);

                    imageSubscriber = imageTransport.subscribe("/ocu/screenshot", 10, &InFieldPicsServer::onNewSnapshot, this);
                    subscriberMetadata = nodeHandle.subscribe("/ocu/screenshot/metadata", 10, &InFieldPicsServer::onNewMetadata, this);
                    
                    subscriberModification = nodeHandle.subscribe(TOPIC_MODIFY, 100, &InFieldPicsServer::onModification, this);

                    cout << endl;
                    cout << "******************" << endl;
                    cout << "InFieldPicsServer" << endl;
                    cout << folder << endl;
                    cout << "Map origin: " << mapOriginLatitude.degrees << "°" << mapOriginLatitude.minutes << "'" << mapOriginLatitude.seconds << "\"N ";
                    cout << mapOriginLongitude.degrees << "°" << mapOriginLongitude.minutes << "'" << mapOriginLongitude.seconds << "\"E @ ";
                    cout << mapOriginDirection << " radians" << endl;
                    cout << "******************" << endl;
                    cout << endl;
                }

                void InFieldPicsServer::run()
                {
                    ros::Rate oneSecond(1);

                    while (ros::ok())
                    {
                        mgr->scanFoldersForNewPictures();

                        if (mgr->getNewPictures().size() == 0)
                        {
                            ROS_INFO("No new pictures");
                        }
                        else
                        {

                            BOOST_FOREACH(const AnnotatedPicture* picture, mgr->getNewPictures())
                            {
                                // Copies to a new message that will not contain the complete file
                                AnnotatedPicture pictureToBeSent;
                                try
                                {
                                    eu::nifti::misc::nifti_pics_server_util::AnnotatedPicUtil::cloneAnnotatedPicture(picture, &pictureToBeSent, false);
                                }
                                catch(const eu::nifti::misc::ExceptionWithString& ex)
                                {
                                    ROS_INFO_STREAM("The loop to publish new pictures caught: " << ex.what());
                                    continue;
                                }
                                
                                ROS_INFO_STREAM("Publishing the latest picture (without complete file): " << pictureToBeSent.filename << " : x " << pictureToBeSent.pose.position.x << " y " << pictureToBeSent.pose.position.y << " z " << pictureToBeSent.pose.position.z);

                                pubNew.publish(pictureToBeSent);
                            }
                        }

                        ros::spinOnce();
                        oneSecond.sleep();
                    }

                    delete mgr;

                }

                bool InFieldPicsServer::onGetAnnotatedPictureRequest(EXIFReader_msgs::GetAnnotatedPicture::Request &request, EXIFReader_msgs::GetAnnotatedPicture::Response &response)
                {
                    //ROS_INFO_STREAM("onGetAnnotatedPicture: " << request.filename);

                    try
                    {
                        response.picture = *mgr->getPicture(request.filename);
                        //ROS_INFO_STREAM("x " << response.picture.x << " y " << response.picture.y);
                    }
                    catch (std::out_of_range) // If the filename does not exist
                    {
                        return false;
                    }

                    return true;
                }

                bool InFieldPicsServer::onGetAllAnnotatedPicturesRequest(EXIFReader_msgs::GetAllAnnotatedPictures::Request &request, EXIFReader_msgs::GetAllAnnotatedPictures::Response &response)
                {
                    //ROS_INFO_STREAM("IN bool InFieldPicsServer::onGetAllAnnotatedPicturesRequest");

                    const MapOfConstAnnotatedPictures& pictures = mgr->getPictures();

                    response.pictures.reserve(pictures.size()); // Creates enough space in the vector

                    // Iterates in reverse order (most recent to latest)
                    for (MapOfConstAnnotatedPictures::const_reverse_iterator it = pictures.rbegin(); it != pictures.rend(); it++)
                    {
                        response.pictures.push_back(AnnotatedPicture());

                        try
                        {
                            eu::nifti::misc::nifti_pics_server_util::AnnotatedPicUtil::cloneAnnotatedPicture(&(*it->second), &response.pictures.back(), request.sendCompleteFiles);
                        }
                        catch (ExceptionWithString& ex)
                        {
                            std::cerr << "Exception in cloneAnnotatedPicture(): " << ex.what() << std::endl;
                            // Todo: handle this better (remove data from response)
                        }

                        //ROS_INFO_STREAM("Added to the response file " << response.pictures.back().filename << " : x " << response.pictures.back().x << " y " << response.pictures.back().y);
                    }
                    
                    assert(mgr->getPictures().size() == response.pictures.size());

                    //ROS_INFO_STREAM("OUT bool InFieldPicsServer::onGetAllAnnotatedPicturesRequest");

                    return true;
                }

                void InFieldPicsServer::onNewSnapshot(const sensor_msgs::ImageConstPtr& imageMsg)
                {
                    stringstream ss;
                    ss << folder << "/FromOCU/NIFTi OCU " << boost::posix_time::to_iso_string(imageMsg->header.stamp.toBoost()) << ".png";
                    string filename = ss.str();
                    
                    if(CVPicUtil::saveImage(imageMsg, ss.str())==false)
                    {
                        ROS_ERROR_STREAM("Problem while trying to save a snapshot coming from the OCU.");
                        return;
                    }
                    
                    ROS_INFO_STREAM("Received and saved image: " << filename);
                }
                
                void InFieldPicsServer::onNewMetadata(const std_msgs::Header& metadataMsg)
                {
                    stringstream ss;
                    ss << folder << "/FromOCU/NIFTi OCU " << boost::posix_time::to_iso_string(metadataMsg.stamp.toBoost()) << ".txt";
                    string filename = ss.str();
                    
                    fs::ofstream file(filename);
                    file << metadataMsg.frame_id;
                    file.close();

                    ROS_INFO_STREAM("Received and saved metadata: " << filename);
                }
                
                void InFieldPicsServer::onModification(const EXIFReader_msgs::Modification& modificationMsg)
                {
                    ROS_INFO_STREAM("Received a modification message for [" << modificationMsg.filename << "]");
                    
                    mgr->modifyPicture(modificationMsg.filename, modificationMsg.annotation);
                    
                }

            }

        }
    }
}
