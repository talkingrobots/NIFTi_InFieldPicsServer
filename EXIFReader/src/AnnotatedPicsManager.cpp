// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2012-12-21

//#include <iostream>
#include <algorithm>

#include <boost/foreach.hpp> 

#include <nifti_pics_server_util/AnnotatedPicUtil.h>
#include <nifti_pics_server_util/EXIFInfo.h>

#include "EXIFParser.h"
#include "FileReadingUtil.h"

#include"AnnotatedPicsManager.h"

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

                AnnotatedPicsManager::AnnotatedPicsManager(const boost::filesystem::path &folder, const GeoCoordinate& mapOriginLatitude, const GeoCoordinate& mapOriginLongitude, const double& mapOriginDirection)
                : mapOriginLatitude(mapOriginLatitude)
                , mapOriginLongitude(mapOriginLongitude)
                , mapOriginDirection(mapOriginDirection)
                , inFieldPicturesLoader(folder, this, mapOriginLatitude, mapOriginLongitude, mapOriginDirection)
                , robotPicturesLoader(folder / "FromOCU", this, mapOriginLatitude, mapOriginLongitude, mapOriginDirection)
                {

                }

                AnnotatedPicsManager::~AnnotatedPicsManager()
                {
                    // Deletes the main pictures and the new pictures

                    for (std::list<const EXIFReader_msgs::AnnotatedPicture*>::iterator it = newPictures.begin(); it != newPictures.end(); it++)
                    {
                        delete *it;
                    }

                    BOOST_FOREACH(MapOfConstAnnotatedPictures::value_type picture, pictures)
                    {
                        delete picture.second;
                    }
                }

                const EXIFReader_msgs::AnnotatedPicture* AnnotatedPicsManager::getPicture(const std::string& filename) const
                {         
                    return pictures.at(filename);
                }
                
                
                void AnnotatedPicsManager::scanFoldersForNewPictures()
                {
                    //cout << "IN void AnnotatedPicsManager::scanFoldersForNewPictures()"  << endl;

                    newPictures.clear();

                    inFieldPicturesLoader.scanFolderForNewPictures();
                    
                    robotPicturesLoader.scanFolderForNewPictures();
                    
                    //cout << "OUT void AnnotatedPicsManager::scanFoldersForNewPictures()" << endl;
                }

                const std::map< const std::string, const EXIFReader_msgs::AnnotatedPicture*>& AnnotatedPicsManager::getPictures() const
                {     
                    return pictures;
                }

                const std::list< const EXIFReader_msgs::AnnotatedPicture*>& AnnotatedPicsManager::getNewPictures() const
                {
                    return newPictures;
                }              
                                
                void AnnotatedPicsManager::addPicture(EXIFReader_msgs::AnnotatedPicture* picture)
                {
                    pictures.insert(MapOfConstAnnotatedPictures::value_type(picture->filename, picture));
                    newPictures.push_back(picture);
                }
                
                void AnnotatedPicsManager::modifyPicture(const std::string& filename, const std::string& annotation)
                {
                    // This is a hack, but it's the only way in C++ to be able to expose a map of const objects.
                    // If I don't do this, then everybody can modify my objects.
                    AnnotatedPicture* pic = const_cast<AnnotatedPicture*> (pictures.at(filename));
                    
                    if(pic == NULL)
                    {
                        cerr << "Received a modification message for a picture that does not exist: " << filename << std::endl;
                        return;
                    }
                    
                    // Updates the live copy and tells no one (there is no need to, because the message was already broadcast on ROS)
                    pic->annotation = annotation;
                    
                    
                    // Here, need to update the copies stored on the drive
                    
                    if(pic->sourceType == AnnotatedPicture::SOURCE_TYPE_EXTERNAL)
                    {
                        inFieldPicturesLoader.modifyPictureAnnotationOnDrive(filename, annotation);
                    }
                    else // pic->sourceType == AnnotatedPicture::SOURCE_TYPE_OCU
                    {
                        robotPicturesLoader.modifyPictureMetadataOnDrive(filename, annotation);
                    }
                    
                }

            }

        }
    }
}
