// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-04-04

#include <algorithm>
#include <string>

#include <boost/foreach.hpp> 

#include "AnnotatedPicsManager.h"
#include "FileReadingUtil.h"

#include "PicturesLoader.h"

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

                PicturesLoader::PicturesLoader(const boost::filesystem::path& folder, AnnotatedPicsManager* picsMgr, const EXIFReader_msgs::GeoCoordinate& mapOriginLatitude, const EXIFReader_msgs::GeoCoordinate& mapOriginLongitude, const double& mapOriginDirection)
                : folder(folder)
                , picsMgr(picsMgr)
                , mapOriginLatitude(mapOriginLatitude)
                , mapOriginLongitude(mapOriginLongitude)
                , mapOriginDirection(mapOriginDirection)
                {
                    // Simple verification
                    if (!fs::exists(folder) || !fs::is_directory(folder))
                    {
                        cerr << "Cannot find the folder. Please check the path. [" << folder << "]" << endl;
                        throw "Cannot find the folder. Please check the path.";
                    }
                }

                void PicturesLoader::discoverNewPicturePaths(std::list<boost::filesystem::path> &newPicturePaths) const
                {
                    std::list<boost::filesystem::path> allPicturesInFolder;

                    // Loads the file names of all pictures in the folder
                    FileReadingUtil::loadPictureNames(folder, allPicturesInFolder); // These are sorted (most recent to oldest)

                    BOOST_FOREACH(fs::path const &picture, allPicturesInFolder)
                    {
                        bool toBeIgnored = find(picturesToIgnore.begin(), picturesToIgnore.end(), picture) != picturesToIgnore.end();
                        if(toBeIgnored)
                            continue;
                        
                        bool alreadyInManagedPictures = picsMgr->getPictures().find(picture.filename().string()) != picsMgr->getPictures().end();
                        if(alreadyInManagedPictures)
                            continue;

                        // Otherwise
                        newPicturePaths.push_back(picture);
                    }

                    // newPicturePaths are sorted (most recent to oldest)
                }

            }

        }
    }
}
