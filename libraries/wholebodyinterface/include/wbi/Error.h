//
//  Error.h
//  wholeBodyInterface
//
//  Created by Francesco Romano on 07/08/15.
//
//

#ifndef IWHOLEBODYINTERFACE_ERROR_H
#define IWHOLEBODYINTERFACE_ERROR_H

#include <string>

namespace wbi {
    class Error;
}

/**
 * @brief Generic Error class
 *
 * This error class supports a domain (i.e. a string specifying the 
 * context of the error. Should be unique), an error code (domain dependent) and an
 * error message to be displayed to the user.
 */
class wbi::Error {
private:
    std::string m_domain; /*<! the error domain */
    int m_errorCode; /*<! the error code */
    std::string m_errorMessage; /*<! the error message */
public:
    /** Empty constructor. Construct an empty error
     */
    Error();

    /**
     * Constructor
     * @param domain the error domain
     * @param code the error code
     * @param message a string message
     */
    Error(std::string domain, int code, std::string message);

    /** Clears the error object
     */
    void clear();

    /** Sets the error message 
     * @param domain the error domain
     * @param code the error code
     * @param message the error message
     */
    void setError(std::string domain, int code, const std::string &message);

    /** Gets  the error domain
     * @return the error domain
     */
    std::string domain() const;

    /** Gets the error code
     * @return the error code
     */
    int errorCode() const;

    /** Gets the error message
     * @return the error message
     */
    std::string errorMessage() const;
};


#endif
