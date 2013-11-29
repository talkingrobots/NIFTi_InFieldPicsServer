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

// Downloaded from http://code.google.com/p/easyexif/
// Modified by Benoit Larochelle for NIFTi (www.nifti.eu)
// 2012-12-19

#ifndef EU_NIFTI_MISC_EXIF_READER_EXIF_PARSER_H
#define EU_NIFTI_MISC_EXIF_READER_EXIF_PARSER_H

#define PARSE_EXIF_OK				               0
#define PARSE_EXIF_ERROR_NO_EXIF				1983
#define PARSE_EXIF_ERROR_UNKNOWN_BYTEALIGN		1984
#define PARSE_EXIF_ERROR_CORRUPT				1985

#include <nifti_pics_server_util/EXIFInfo.h>

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                class EXIFParser
                {
                public:

                    //
                    // Parse basic EXIF information from a JPEG file buffer.
                    //
                    // IN: 		const char* to binary JPEG data
                    // RETURN:	0 on succes with 'result' filled out
                    //			error code otherwise, as defined by the 
                    //				PARSE_EXIF_ERROR_* macros
                    //
                    static int parseEXIF(unsigned char *JPEGfile, unsigned int length, eu::nifti::misc::nifti_pics_server_util::EXIFInfo &result);
                    
                private:
                    EXIFParser(unsigned char *JPEGfile, unsigned int length, eu::nifti::misc::nifti_pics_server_util::EXIFInfo &result);
                    
                    int run();
                    
                    int parseTIFFHeader();
                    int parse0thIFD();
                    int parseSubIFD();
                    int parseGPSIFD();
                    
                    void parseEntry(const unsigned char *entry, unsigned short &tag, unsigned short &type, unsigned &count, unsigned &valueOffset);
                    
                    void handleIFDEntry(unsigned short tag, unsigned short type, unsigned count, unsigned valueOffset);
                    void handleSubIFDEntry(unsigned short tag, unsigned short type, unsigned count, unsigned valueOffset);
                    void handleGPSIFDEntry(unsigned short tag, unsigned short type, unsigned count, unsigned valueOffset);
                    
                    unsigned char *buf;
                    unsigned bufferLength;
                    eu::nifti::misc::nifti_pics_server_util::EXIFInfo &result;
                    
                    unsigned START_OF_EXIF_HEADER;
                    unsigned START_OF_TIFF_HEADER;
                    
                    unsigned OFFSET_0TH_IFD;
                    unsigned OFFSET_EXIF_SUB_ID;
                    unsigned OFFSET_GPS_IFD;
                    
                    bool LITTLE_ENDIAN_INTEL; // byte alignment
                    
                    // Page 13 in the specs
                    const unsigned short FIELD_TAG;
                    const unsigned short FIELD_TYPE;
                    const unsigned short FIELD_COUNT;
                    const unsigned short FIELD_VALUE_OFFSET;
                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_EXIF_PARSER_H
