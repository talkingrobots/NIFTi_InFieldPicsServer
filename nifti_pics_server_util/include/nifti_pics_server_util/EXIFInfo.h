/**************************************************************************
  exif.h  -- A simple ISO C++ library to parse basic EXIF 
             information from a JPEG file.

  Based on the description of th EXIF file format at:
  http://park2.wakwak.com/~tsuruzoh/Computer/Digicams/exif-e.html  

  Copyright (c) 2010 Mayank Lahiri
  mlahiri@gmail.com
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are met:

  -- Redistributions of source code must retain the above copyright notice, 
     this list of conditions and the following disclaimer.
  -- Redistributions in binary form must reproduce the above copyright notice, 
     this list of conditions and the following disclaimer in the documentation 
     and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN 
   NO EVENT SHALL THE FREEBSD PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
   OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Modified by Benoit Larochelle for NIFTi (www.nifti.eu)
// 2012-12-19

#ifndef EU_NIFTI_MISC_EXIF_READER_EXIF_INFO_H
#define EU_NIFTI_MISC_EXIF_READER_EXIF_INFO_H

#include <EXIFReader_msgs/GeoCoordinate.h>

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace nifti_pics_server_util
            {

                // If any values are 0 or NULL (with the exception of byteAlign), then
                // those values are NOT present in the EXIF information and should be
                // ignored.

                class EXIFInfo
                {
                public:

                    EXIFInfo()
                    {
                        reset();
                    };

                    ~EXIFInfo()
                    {
                        if (imgDescription)
                            delete[] imgDescription;
                        if (cameraMake)
                            delete[] cameraMake;
                        if (cameraModel)
                            delete[] cameraModel;
                        if (dateTimeModified)
                            delete[] dateTimeModified;
                        if (dateTimeOriginal)
                            delete[] dateTimeOriginal;
                    };

                    void reset()
                    {
                        cameraMake = cameraModel = dateTimeModified = dateTimeOriginal = imgDescription = (char*) 0;
                        focalLength = focalLength35mm = 0;
                        fieldOfViewHorizontal = fieldOfViewVertical = FStop = exposureTime = 0;

                        gpsLatitude = EXIFReader_msgs::GeoCoordinate();
                        gpsLongitude = EXIFReader_msgs::GeoCoordinate();
                        gpsAltitude = 0;
                        gpsImgDirection = 0;
                    }


                    char byteAlign; // 0 = Motorola byte alignment, 1 = Intel 
                    char *cameraMake; // String with camera manufacturer's name
                    char *cameraModel; // String with camera model
                    char *dateTimeModified; // date/time string of last modification 
                    // (may be blank)
                    char *dateTimeOriginal; // date/time string of original image 
                    // (may be blank, or not present)
                    char *imgDescription; // String describing the image
                    unsigned focalLength; // Focal length of lens (millimeters)
                    unsigned short focalLength35mm; // Focal length of lens (millimeters) (35mm equivalent)
                    double fieldOfViewHorizontal; // Field of view (in radians)
                    double fieldOfViewVertical; // Field of view (in radians)
                    double FStop; // F-number of lens = 1/FStop 
                    double exposureTime; // Exposure time in seconds

                    EXIFReader_msgs::GeoCoordinate gpsLatitude;
                    EXIFReader_msgs::GeoCoordinate gpsLongitude;
                    double gpsAltitude;
                    double gpsImgDirection;
                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_EXIF_INFO_H
