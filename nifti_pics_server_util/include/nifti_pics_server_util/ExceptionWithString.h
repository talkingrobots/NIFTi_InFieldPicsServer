// Benoit 2012-10-22

#include <exception>
#include <string>

#ifndef EU_NIFTI_MISC_EXCEPTION_WITH_STRING_H
#define EU_NIFTI_MISC_EXCEPTION_WITH_STRING_H

namespace eu
{
    namespace nifti
    {
        namespace misc
        {

            class ExceptionWithString : public std::exception
            {
            public:

                ExceptionWithString(std::string msg) throw ();
                virtual ~ExceptionWithString() throw ();

                virtual const char* what() const throw ();

            protected:

                std::string msg;

            };

        }
    }
}

#endif //EU_NIFTI_MISC_EXCEPTION_WITH_STRING_H
