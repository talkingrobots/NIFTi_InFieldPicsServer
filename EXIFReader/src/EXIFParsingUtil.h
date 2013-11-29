/**************************************************************************
  exif.cpp  -- A simple ISO C++ library to parse basic EXIF 
               information from a JPEG file.

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

#ifndef EU_NIFTI_MISC_EXIF_READER_EXIF_PARSING_UTIL_H
#define EU_NIFTI_MISC_EXIF_READER_EXIF_PARSING_UTIL_H 

#include <cstring>
#include <math.h>

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                class EXIFParsingUtil
                {
                public: 
                    
                    // Returns a 32-bit unsigned int from a binary buffer
                    static inline unsigned int parse32(const unsigned char *buf, bool little_endian_intel)
                    {
                        return little_endian_intel ?
                                (((unsigned) buf[3] << 24) + ((unsigned) buf[2] << 16) + ((unsigned) buf[1] << 8) + buf[0])
                                :
                                (((unsigned) buf[0] << 24) + ((unsigned) buf[1] << 16) + ((unsigned) buf[2] << 8) + buf[3]);
                    }

                    // Returns a 16-bit unsigned short from a binary buffer
                    static inline unsigned short parse16(const unsigned char *buf, bool little_endian_intel)
                    {
                        return little_endian_intel ?
                                (((unsigned) buf[1] << 8) + buf[0])
                                :
                                (((unsigned) buf[0] << 8) + buf[1]);
                    }

                    // Copies a string from data (binary) into a buffer
                    static inline void copyEXIFString(char **dest, unsigned ncomp, unsigned base, unsigned offs, unsigned char *buf)
                    {
                        *dest = new char[ncomp + 1];
                        memset(*dest, 0, ncomp + 1);
                        if (ncomp > 4)
                            memcpy(*dest, (char*) (buf + base + offs), ncomp);
                        else
                            memcpy(*dest, (char*) &offs, 4);
                    }

                    // Returns a double representing a rational number from two 32-bit binary numbers stored one after the other
                    static inline double parseEXIFrational(unsigned char *buf, bool little_endian_intel)
                    {
                        double numerator = parse32(buf, little_endian_intel);
                        double denominator = parse32(buf + 4, little_endian_intel);
                        if (denominator == 0)
                        {
                            return INFINITY;
                        }
                        else
                        {
                            return numerator / denominator;
                        }
                        
                        // Benoit: old code from the web. I don't know why he returned that. Also, the function used to return a float
                        //if (denominator < 1e-20)
                        //    return 0;
                    }

                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_EXIF_PARSING_UTIL_H