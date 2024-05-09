#ifndef SLAMEXCEPTION_H
#define SLAMEXCEPTION_H

#include <iostream>
#include <exception>

namespace slam
{

    // Define custom exception class
    class SLAMException : public std::exception {
    public:
        // Constructor to initialize the exception with an error message
        SLAMException(const char* message) 
        : errorMessage(message) {}

        // Override the what() method to return the error message
        virtual const char* what() const throw()
        {
            return errorMessage;
        }

    private:
        const char* errorMessage;
    };

}

#endif