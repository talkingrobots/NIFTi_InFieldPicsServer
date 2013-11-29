// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2012-12-19

#ifndef EU_NIFTI_MISC_EXIF_READER_FILE_READING_UTIL_H
#define EU_NIFTI_MISC_EXIF_READER_FILE_READING_UTIL_H 

#include <list>

#include <boost/filesystem.hpp>

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                class FileReadingUtil
                {
                public:

                    static int loadFile(const char *filePath, unsigned char *&buffer, unsigned long &fileSize);

                    static void loadPictureNames(const boost::filesystem::path &folderPath, std::list<boost::filesystem::path> &picturePaths);
                    
                private:
                    
                    static void listFilesInDirectory(const boost::filesystem::path& directory, std::list<boost::filesystem::path>& files);
                   
                };

            }
        }
    }
}

#endif // EU_NIFTI_MISC_EXIF_READER_FILE_READING_UTIL_H