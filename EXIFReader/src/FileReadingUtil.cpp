// Benoit Larochelle for NIFTi (www.nifti.eu)
// 2012-12-19

#include <stdio.h>

#include <boost/foreach.hpp> 

#include "FileReadingUtil.h"

using namespace std;
namespace fs = boost::filesystem;

namespace eu
{
    namespace nifti
    {
        namespace misc
        {
            namespace EXIFReader
            {

                int FileReadingUtil::loadFile(const char *filePath, unsigned char *&buffer, unsigned long &fileSize)
                {
                    // Reads the file into a buffer
                    FILE *fp = fopen(filePath, "rb");
                    if (!fp)
                    {
                        printf("Can't open file: %s\n", filePath);
                        return -1;
                    }

                    fseek(fp, 0, SEEK_END);
                    fileSize = ftell(fp);
                    rewind(fp);

                    buffer = new unsigned char[fileSize];
                    if (fread(buffer, 1, fileSize, fp) != fileSize)
                    {
                        printf("Can't read file: %s\n", filePath);
                        return -2;
                    }

                    fclose(fp);

                    return 0;
                }

                void FileReadingUtil::loadPictureNames(const boost::filesystem::path &folderPath, std::list<boost::filesystem::path> &picturePaths)
                {

                    try
                    {

                        std::list<boost::filesystem::path> allFilePaths;
                        listFilesInDirectory(folderPath, allFilePaths);

                        // Iterates over all files (http://stackoverflow.com/questions/8725331/iterate-over-all-files-in-a-directory-using-boost-foreach)

                        BOOST_FOREACH(fs::path const &p, allFilePaths)
                        {
                            if (is_regular_file(p) && (p.extension() == ".JPG" || p.extension() == ".jpg" || p.extension() == ".png"))
                            {
                                // At this point, a picture has been found
                                //cout << "Discovered picture: " << p.filename() << endl;

                                picturePaths.push_front(p);
                            }
                        }

                    }
                    catch (const boost::filesystem::filesystem_error& ex)
                    {
                        std::cerr << ex.what() << std::endl;
                    }
                    
                    picturePaths.sort();
                    picturePaths.reverse();

                }

                void FileReadingUtil::listFilesInDirectory(const boost::filesystem::path& directory, std::list<boost::filesystem::path>& files)
                {
                    files.clear();

                    boost::filesystem::directory_iterator beg(directory);
                    boost::filesystem::directory_iterator end;
                    std::copy(beg, end, std::inserter(files, files.begin()));
                }


            }
        }
    }
}
