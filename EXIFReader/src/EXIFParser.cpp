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

// Downloaded from http://code.google.com/p/easyexif/
// Modified by Benoit Larochelle for NIFTi (www.nifti.eu)
// 2012-12-19


// IFD: Image File Directory

// The first two bytes at beginning of IFD contain the number of IFD entries (items inside IFD). 
// After that value, actual entries start. 
// Each entry is always 12 bytes long.

#define EXIF_PARSER_DEBUG 0

#include <math.h>
#include <iostream>

#include "EXIFParsingUtil.h"
#include "EXIFSpec.h"

#include"EXIFParser.h"

using namespace std;
using namespace eu::nifti::misc::nifti_pics_server_util;

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                const double SENSOR_SIZE_35MM_HORIZONTAL = 36;
                const double SENSOR_SIZE_35MM_VERTICAL = 24;
                
                EXIFParser::EXIFParser(unsigned char *buf, unsigned bufferLength, EXIFInfo &result)
                : buf(buf)
                , bufferLength(bufferLength)
                , result(result)
                , START_OF_EXIF_HEADER(0)
                , START_OF_TIFF_HEADER(0)
                , OFFSET_0TH_IFD(0)
                , OFFSET_EXIF_SUB_ID(0)
                , OFFSET_GPS_IFD(0)
                , FIELD_TAG(0)
                , FIELD_TYPE(2)
                , FIELD_COUNT(4)
                , FIELD_VALUE_OFFSET(8)
                {

                }

                int EXIFParser::parseEXIF(unsigned char *buf, unsigned bufferLength, EXIFInfo &result)
                {
                    EXIFParser p(buf, bufferLength, result);
                    return p.run();
                }

                int EXIFParser::run()
                {
                    EXIFParser(buf, bufferLength, result);
                    if (bufferLength == 0)
                        return PARSE_EXIF_ERROR_NO_EXIF;

                    if (EXIF_PARSER_DEBUG)
                        cout << "bufferLength: " << bufferLength << endl;

                    result.reset();

                    unsigned offset = 0; // current offset into buffer

                    // Scan for EXIF header
                    for (offset = 0; offset < bufferLength - 1; offset++)
                    {
                        if (buf[offset] == 0xFF && buf[offset + 1] == 0xE1)
                        {
                            break;
                        }
                    }
                    if (offset == bufferLength - 1) // No header found
                        return PARSE_EXIF_ERROR_NO_EXIF;

                    START_OF_EXIF_HEADER = offset;

                    if (EXIF_PARSER_DEBUG)
                        cout << "START_OF_EXIF_HEADER: " << START_OF_EXIF_HEADER << endl;

                    // Benoit: I don't know what that is
                    offset += 4;
                    if (buf[offset] != 0x45 || buf[offset + 1] != 0x78 || buf[offset + 2] != 0x69)
                        return PARSE_EXIF_ERROR_NO_EXIF;


                    int errorCode = PARSE_EXIF_OK;

                    errorCode = parseTIFFHeader();

                    if (errorCode != PARSE_EXIF_OK)
                        return errorCode;

                    errorCode = parse0thIFD();

                    if (errorCode != PARSE_EXIF_OK)
                        return errorCode;

                    if (OFFSET_EXIF_SUB_ID >= bufferLength || OFFSET_GPS_IFD >= bufferLength)
                    {
                        if (EXIF_PARSER_DEBUG)
                            cout << "subIFD offset greater than buffer length" << endl;

                        return PARSE_EXIF_ERROR_CORRUPT;
                    }


                    if (OFFSET_EXIF_SUB_ID)
                    {
                        errorCode = parseSubIFD();

                        if (errorCode != PARSE_EXIF_OK)
                            return errorCode;

                        if (OFFSET_GPS_IFD)
                        {
                            errorCode = parseGPSIFD();
                            if (errorCode != PARSE_EXIF_OK)
                                return errorCode;
                        }
                    }

                    if (EXIF_PARSER_DEBUG)
                        cout << "Finished scanning" << endl;

                    return errorCode;
                }

                int EXIFParser::parseTIFFHeader()
                {
                    START_OF_TIFF_HEADER = START_OF_EXIF_HEADER + 10; // 10 bytes further than the EXIF header. Why? I don't know.

                    if (EXIF_PARSER_DEBUG)
                        cout << "START_OF_TIFF_HEADER: " << START_OF_TIFF_HEADER << endl;

                    const unsigned FIELD_BYTE_ORDER = START_OF_TIFF_HEADER;
                    const unsigned FIELD_FORTY_TWO = START_OF_TIFF_HEADER + 2;
                    const unsigned FIELD_OFFSET_0TH_IFD = START_OF_TIFF_HEADER + 4;



                    // Get byte alignment (Motorola or Intel)
                    if (buf[FIELD_BYTE_ORDER] == 0x49 && buf[FIELD_BYTE_ORDER + 1] == 0x49) // II
                        LITTLE_ENDIAN_INTEL = true;
                    else if (buf[FIELD_BYTE_ORDER] == 0x4d && buf[FIELD_BYTE_ORDER + 1] == 0x4d) // MM
                        LITTLE_ENDIAN_INTEL = false;
                    else
                        return PARSE_EXIF_ERROR_UNKNOWN_BYTEALIGN;

                    result.byteAlign = LITTLE_ENDIAN_INTEL;

                    if (EXIF_PARSER_DEBUG)
                        cout << "Alignment is: " << LITTLE_ENDIAN_INTEL << endl;


                    // Simple verification. Field "42" contains value 42 (TIFF signature)
                    if (EXIFParsingUtil::parse16(buf + FIELD_FORTY_TWO, LITTLE_ENDIAN_INTEL) != 42)
                    {
                        return PARSE_EXIF_ERROR_CORRUPT;
                    }


                    // Get offset into 0th IFD
                    OFFSET_0TH_IFD = EXIFParsingUtil::parse32(buf + FIELD_OFFSET_0TH_IFD, LITTLE_ENDIAN_INTEL);

                    if (EXIF_PARSER_DEBUG)
                        cout << "OFFSET_OF_0TH_IFD: " << OFFSET_0TH_IFD << endl;


                    // Verification (from original code)
                    if (START_OF_TIFF_HEADER + OFFSET_0TH_IFD >= bufferLength)
                    {
                        return PARSE_EXIF_ERROR_CORRUPT;
                    }

                    return PARSE_EXIF_OK;
                }

                int EXIFParser::parse0thIFD()
                {
                    // Find the number of IFD entries
                    const unsigned short NUM_IFD_ENTRIES = EXIFParsingUtil::parse16(buf + START_OF_TIFF_HEADER + OFFSET_0TH_IFD, LITTLE_ENDIAN_INTEL);

                    if (EXIF_PARSER_DEBUG)
                        cout << "NUM_IFD_ENTRIES: " << NUM_IFD_ENTRIES << endl;


                    const unsigned START_FIRST_ENTRY = START_OF_TIFF_HEADER + OFFSET_0TH_IFD + 2;


                    unsigned char *entryPointer;
                    unsigned short tag;
                    unsigned short type;
                    unsigned count;
                    unsigned valueOffset;

                    // Jump to the first IFD, scan tags there.
                    for (unsigned currentEntry = 0; currentEntry < NUM_IFD_ENTRIES; currentEntry++)
                    {
                        entryPointer = buf + START_FIRST_ENTRY + (12 * currentEntry); // Each entry is 12 bytes

                        parseEntry(entryPointer, tag, type, count, valueOffset);

                        handleIFDEntry(tag, type, count, valueOffset);

                    }

                    if (EXIF_PARSER_DEBUG)
                        cout << "Finished scanning the IFDs" << endl;

                    return PARSE_EXIF_OK;
                }

                int EXIFParser::parseSubIFD()
                {
                    const unsigned short NUM_SUB_IFDS = EXIFParsingUtil::parse16(buf + OFFSET_EXIF_SUB_ID, LITTLE_ENDIAN_INTEL);

                    if (EXIF_PARSER_DEBUG)
                        cout << "Scanning SubIFDs at offset " << OFFSET_EXIF_SUB_ID << ". There are " << NUM_SUB_IFDS << endl;

                    const unsigned START_FIRST_SUB_ENTRY = OFFSET_EXIF_SUB_ID + 2;

                    unsigned char *entryPointer;
                    unsigned short tag;
                    unsigned short type;
                    unsigned count;
                    unsigned valueOffset;

                    for (unsigned short currentEntry = 0; currentEntry < NUM_SUB_IFDS; currentEntry++)
                    {
                        entryPointer = buf + START_FIRST_SUB_ENTRY + (12 * currentEntry); // Each entry is 12 bytes

                        parseEntry(entryPointer, tag, type, count, valueOffset);

                        handleSubIFDEntry(tag, type, count, valueOffset);

                    }

                    if (EXIF_PARSER_DEBUG)
                        cout << "Finished scanning SubIFDs" << endl;

                    return PARSE_EXIF_OK;
                }

                int EXIFParser::parseGPSIFD()
                {
                    // http://www.exif.org/Exif2-2.PDF see p.46.
                    const unsigned short NUM_GPS_IFD_ENTRIES = EXIFParsingUtil::parse16(buf + OFFSET_GPS_IFD, LITTLE_ENDIAN_INTEL);

                    if (EXIF_PARSER_DEBUG)
                        cout << "Scanning GPS IFDs at offset " << OFFSET_GPS_IFD << ". There are " << NUM_GPS_IFD_ENTRIES << " GPS entries" << endl;

                    const unsigned START_FIRST_GPS_ENTRY = OFFSET_GPS_IFD + 2;

                    unsigned char *entryPointer;
                    unsigned short tag;
                    unsigned short type;
                    unsigned count;
                    unsigned valueOffset;

                    for (unsigned short currentEntry = 0; currentEntry < NUM_GPS_IFD_ENTRIES; currentEntry++)
                    {
                        entryPointer = buf + START_FIRST_GPS_ENTRY + (12 * currentEntry); // Each entry is 12 bytes

                        parseEntry(entryPointer, tag, type, count, valueOffset);

                        handleGPSIFDEntry(tag, type, count, valueOffset);
                    }

                    return PARSE_EXIF_OK;
                }

                void EXIFParser::parseEntry(const unsigned char *entry, unsigned short &tag, unsigned short &type, unsigned &count, unsigned &valueOffset)
                {
                    tag = EXIFParsingUtil::parse16(entry + FIELD_TAG, LITTLE_ENDIAN_INTEL);
                    type = EXIFParsingUtil::parse16(entry + FIELD_TYPE, LITTLE_ENDIAN_INTEL);
                    count = EXIFParsingUtil::parse32(entry + FIELD_COUNT, LITTLE_ENDIAN_INTEL);
                    valueOffset = EXIFParsingUtil::parse32(entry + FIELD_VALUE_OFFSET, LITTLE_ENDIAN_INTEL);
                }

                void EXIFParser::handleIFDEntry(unsigned short tag, unsigned short type, unsigned count, unsigned valueOffset)
                {

                    if (EXIF_PARSER_DEBUG)
                        cout << "\tScanned IFD: ";

                    switch (tag)
                    {
                        case EXIF_ExifOffset:
                            // EXIF subIFD offset
                            if (EXIF_PARSER_DEBUG) cout << "EXIF subIFD offset: ";
                            OFFSET_EXIF_SUB_ID = START_OF_TIFF_HEADER + valueOffset;
                            if (EXIF_PARSER_DEBUG) cout << OFFSET_EXIF_SUB_ID << endl;
                            break;

                        case EXIF_Make:
                            // Digicam manufacturer
                            if (EXIF_PARSER_DEBUG) cout << "Digicam manufacturer: ";
                            EXIFParsingUtil::copyEXIFString(&result.cameraMake, count, START_OF_TIFF_HEADER, valueOffset, buf);
                            if (EXIF_PARSER_DEBUG) cout << result.cameraMake << endl;
                            break;

                        case EXIF_Model:
                            // Digicam model
                            if (EXIF_PARSER_DEBUG) cout << "Digicam model: ";
                            EXIFParsingUtil::copyEXIFString(&result.cameraModel, count, START_OF_TIFF_HEADER, valueOffset, buf);
                            if (EXIF_PARSER_DEBUG) cout << result.cameraModel << endl;
                            break;

                        case EXIF_DateTime:
                            // EXIF/TIFF date/time of image
                            if (EXIF_PARSER_DEBUG) cout << "EXIF/TIFF date/time of image: ";
                            EXIFParsingUtil::copyEXIFString(&result.dateTimeModified, count, START_OF_TIFF_HEADER, valueOffset, buf);
                            if (EXIF_PARSER_DEBUG) cout << result.dateTimeModified << endl;
                            break;

                        case EXIF_ImageDescription:
                            // image description 
                            if (EXIF_PARSER_DEBUG) cout << "image description: ";
                            EXIFParsingUtil::copyEXIFString(&result.imgDescription, count, START_OF_TIFF_HEADER, valueOffset, buf);
                            if (EXIF_PARSER_DEBUG) cout << result.imgDescription << endl;
                            break;

                        case EXIF_GPSInfo:
                            // EXIF gpsIFD offset
                            if (EXIF_PARSER_DEBUG) cout << "gpsIFDOffset: ";
                            OFFSET_GPS_IFD = START_OF_TIFF_HEADER + valueOffset;
                            if (EXIF_PARSER_DEBUG) cout << OFFSET_GPS_IFD << endl;
                            break;

                        case EXIF_ImageWidth:
                            if (EXIF_PARSER_DEBUG) cout << "EXIF_ImageWidth: ";

                            if (EXIF_PARSER_DEBUG) cout << valueOffset << endl;
                            break;

                        case EXIF_ImageLength:
                            if (EXIF_PARSER_DEBUG) cout << "EXIF_ImageLength: ";

                            if (EXIF_PARSER_DEBUG) cout << valueOffset << endl;
                            break;

                        default:
                            if (EXIF_PARSER_DEBUG)
                            {
                                cout << hex;
                                cout << "Other: 0x" << tag << endl;
                                cout << dec;
                            }
                    }

                }

                void EXIFParser::handleSubIFDEntry(unsigned short tag, unsigned short type, unsigned count, unsigned valueOffset)
                {
                    if (EXIF_PARSER_DEBUG)
                        cout << "\tScanned subIFD: ";

                    switch (tag)
                    {
                        case EXIF_DateTimeOriginal:
                            if (EXIF_PARSER_DEBUG) cout << "original image date/time string" << endl;
                            EXIFParsingUtil::copyEXIFString(&result.dateTimeOriginal, count, START_OF_TIFF_HEADER, valueOffset, buf);
                            break;

                        case EXIF_FocalLength:
                            if (EXIF_PARSER_DEBUG) cout << "Focal length in mm" << endl;
                            // result.focalLength = *((unsigned*)(buf+ifdOffset+coffs));
                            result.focalLength = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset, LITTLE_ENDIAN_INTEL);
                            break;

                        case EXIF_FNumber:
                            if (EXIF_PARSER_DEBUG) cout << "F-stop" << endl;
                            result.FStop = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset, LITTLE_ENDIAN_INTEL);
                            break;

                        case EXIF_ExposureTime:
                            if (EXIF_PARSER_DEBUG) cout << "Exposure time" << endl;
                            result.exposureTime = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset, LITTLE_ENDIAN_INTEL);
                            break;

                        case EXIF_FocalLengthIn35mmFilm:
                            if (EXIF_PARSER_DEBUG) cout << "Focal length in mm (35mm equivalent)" << endl;

                            if (!LITTLE_ENDIAN_INTEL)
                            {
                                // Need to shift the value before it was parsed as a 32-bit integer but it is 16-bit integer
                                valueOffset = valueOffset >> 16;
                            }
                          
                            result.focalLength35mm = valueOffset;
                            result.fieldOfViewHorizontal = 2 * atan2(SENSOR_SIZE_35MM_HORIZONTAL, 2 * result.focalLength35mm);
                            result.fieldOfViewVertical = 2 * atan2(SENSOR_SIZE_35MM_VERTICAL, 2 * result.focalLength35mm);

                            break;

                        default:
                            if (EXIF_PARSER_DEBUG)
                            {
                                cout << hex;
                                cout << "Other: 0x" << tag << endl;
                                cout << dec;
                            }
                    }

                }

                void EXIFParser::handleGPSIFDEntry(unsigned short tag, unsigned short type, unsigned count, unsigned valueOffset)
                {
                    if (EXIF_PARSER_DEBUG)
                        cout << "\tScanned gpsIFD: ";

                    switch (tag)
                    {
                        case EXIF_GPSLatitude:
                            if (EXIF_PARSER_DEBUG) cout << "Latitude" << endl;

                            result.gpsLatitude.degrees = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset + 0, LITTLE_ENDIAN_INTEL);
                            result.gpsLatitude.minutes = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset + 8, LITTLE_ENDIAN_INTEL);
                            result.gpsLatitude.seconds = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset + 16, LITTLE_ENDIAN_INTEL);
                            break;
                        case EXIF_GPSLongitude:
                            if (EXIF_PARSER_DEBUG) cout << "Longitude" << endl;

                            result.gpsLongitude.degrees = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset + 0, LITTLE_ENDIAN_INTEL);
                            result.gpsLongitude.minutes = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset + 8, LITTLE_ENDIAN_INTEL);
                            result.gpsLongitude.seconds = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset + 16, LITTLE_ENDIAN_INTEL);
                            break;
                        case EXIF_GPSAltitude:
                            if (EXIF_PARSER_DEBUG) cout << "Altitude" << endl;

                            result.gpsAltitude = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset + 0, LITTLE_ENDIAN_INTEL);
                            break;
                            
                        case EXIF_GPSImgDirection: // Benoit: This was never tested because I don't have any pictures with the direction embedded
                            if (EXIF_PARSER_DEBUG) cout << "Direction" << endl;

                            result.gpsImgDirection = EXIFParsingUtil::parseEXIFrational(buf + START_OF_TIFF_HEADER + valueOffset + 0, LITTLE_ENDIAN_INTEL);
                            break;

                        default:
                            if (EXIF_PARSER_DEBUG)
                            {
                                cout << hex;
                                cout << "Other: 0x" << tag << endl;
                                cout << dec;
                            }
                    }

                }

            }
        }
    }
}