//
//  Error.cpp
//  wholeBodyInterface
//
//  Created by Francesco Romano on 07/08/15.
//
//

#include "Error.h"

wbi::Error::Error(std::string domain, int code, std::string message)
: m_domain(domain)
, m_errorCode(code)
, m_errorMessage(message) {}

void wbi::Error::clear() {
    m_domain = -1;
    m_errorCode = -1;
    m_errorMessage = "";
}
void wbi::Error::setError(std::string domain, int code, const std::string &message)
{
    m_domain = domain;
    m_errorCode = code;
    m_errorMessage = message;
}

std::string wbi::Error::domain() const {
    return m_domain;
}

int wbi::Error::errorCode() const {
    return m_errorCode;
}

std::string wbi::Error::errorMessage() const {
    return m_errorMessage;
}