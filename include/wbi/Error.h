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

class wbi::Error {
private:
    std::string m_domain;
    int m_errorCode;
    std::string m_errorMessage;
public:
    Error(std::string domain, int code, std::string message);
    void clear();
    void setError(std::string domain, int code, const std::string &message);

    std::string domain() const;
    int errorCode() const;
    std::string errorMessage() const;
};


#endif
