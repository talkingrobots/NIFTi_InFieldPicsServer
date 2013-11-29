// Benoit 2012-10-22

#include "nifti_pics_server_util/ExceptionWithString.h"

namespace eu
{
    namespace nifti
    {
        namespace misc
        {

            ExceptionWithString::ExceptionWithString(std::string msg) throw ()
            : exception()
            , msg(msg)
            {

            }

            ExceptionWithString::~ExceptionWithString() throw ()
            {

            }

            const char* ExceptionWithString::what() const throw ()
            {
                return msg.c_str();
            }
        }
    }
}

