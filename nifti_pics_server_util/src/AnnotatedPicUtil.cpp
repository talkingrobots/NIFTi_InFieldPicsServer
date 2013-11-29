// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2013-02-18

#include <boost/foreach.hpp> 

#include "nifti_pics_server_util/ExceptionWithString.h"
#include "nifti_pics_server_util/EXIFInfo.h"

#include "nifti_pics_server_util/AnnotatedPicUtil.h"

using namespace std;
using namespace EXIFReader_msgs;

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace nifti_pics_server_util
            {
                const double RAD_TO_DEG = 57.2957795130;
               
                void AnnotatedPicUtil::printOutAnnotatedPictureDetails(const std::list<AnnotatedPicture*> &pictures)
                {
                    BOOST_FOREACH(const AnnotatedPicture* picture, pictures)
                    {
                        printOutAnnotatedPictureDetails(picture);
                    }
                }

                void AnnotatedPicUtil::printOutAnnotatedPictureDetails(const AnnotatedPicture* picture)
                {
                    cout << "Picture: " << picture->filename << endl;

                    if (!picture->dateTime.isZero())
                        cout << "\tDate and Time (ROS): " << picture->dateTime << endl;
                    
                    if (picture->dateTimeFormatted != "")
                        cout << "\tDate and Time (formatted): " << picture->dateTimeFormatted << endl;

                    if (picture->fieldOfViewHorizontal != 0)
                    {
                        cout << "\tField of View (horizontal): " << picture->fieldOfViewHorizontal << " radians (" << picture->fieldOfViewHorizontal * RAD_TO_DEG << " degrees)" << endl;
                        cout << "\tField of View (vertical): " << picture->fieldOfViewVertical << " radians (" << picture->fieldOfViewVertical * RAD_TO_DEG << " degrees)" << endl;
                    }

                    if (picture->latitude.degrees != 0)
                    {
                        cout << "\tGPS Latitude: " << picture->latitude.degrees << " " << picture->latitude.minutes << " " << picture->latitude.seconds << endl;
                        cout << "\tGPS Longitude: " << picture->longitude.degrees << " " << picture->longitude.minutes << " " << picture->longitude.seconds << endl;
                    }

                    if (picture->direction != 0)
                    {
                        cout << "\tDirection: " << picture->direction << " degrees (" << picture->direction / RAD_TO_DEG << " radians) clockwise from North" << endl;
                    }
                }

                EXIFReader_msgs::AnnotatedPicture* AnnotatedPicUtil::cloneAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* source, bool copyCompleteFile)
                {
                    AnnotatedPicture* destination = new AnnotatedPicture();
                    cloneAnnotatedPicture(source, destination, copyCompleteFile);
                    return destination;
                }

                void AnnotatedPicUtil::cloneAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* source, EXIFReader_msgs::AnnotatedPicture* destination, bool copyCompleteFile)
                {
                    if (source == NULL)
                    {
                        throw eu::nifti::misc::ExceptionWithString("The parameter \'source\' is null");
                    }
                    if (destination == NULL)
                    {
                        throw eu::nifti::misc::ExceptionWithString("The parameter \'destination\' is null");
                    }
                    
                    destination->filename = source->filename;
                    destination->dateTime = source->dateTime;
                    destination->dateTimeFormatted = source->dateTimeFormatted;
                    destination->source = source->source;
                    destination->annotation = source->annotation;
                    destination->latitude = source->latitude;
                    destination->longitude = source->longitude;
                    destination->pose = source->pose;
                    destination->direction = source->direction;
                    destination->fieldOfViewHorizontal = source->fieldOfViewHorizontal;
                    destination->fieldOfViewVertical = source->fieldOfViewVertical;
                    

                    if (copyCompleteFile) // Does not necessarily copy the complete file (it would be a few MBs)
                    {
                        destination->completeFile.resize(source->completeFile.size());
                        memcpy(destination->completeFile.data(), source->completeFile.data(), source->completeFile.size());
                        //destination->completeFile = source->completeFile;           
                    }
                }

            }

        }
    }
}
