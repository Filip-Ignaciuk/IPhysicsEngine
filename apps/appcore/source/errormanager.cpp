#include "errormanager.hpp"

IApp::Error::Error(ErrorSeverity _errorSeverity, std::string& _errorTitle, std::string& _errorMessage) : m_errorSeverity(_errorSeverity), m_errorTitle(_errorTitle), m_errorMessage(_errorMessage) {}

IApp::ErrorSeverity IApp::Error::GetErrorSeverity() const{
    return m_errorSeverity;
}

std::string IApp::Error::GetErrorTitle() const{
    return m_errorTitle;
}

std::string IApp::Error::GetErrorMessage() const{
    return m_errorMessage;
}

bool IApp::Error::operator<(const Error& _other) const{
    return this->m_errorSeverity < _other.m_errorSeverity;
}

bool IApp::Error::operator==(const Error& _other) const{
    return this->m_errorSeverity == _other.m_errorSeverity && this->m_errorTitle.compare(_other.m_errorTitle) == 0 && this->m_errorMessage.compare(_other.m_errorMessage) == 0;
}

std::priority_queue<IApp::Error> IApp::ErrorManager::m_errors;

void IApp::ErrorManager::AddError(Error& _error){
    m_errors.emplace(_error);
}

bool IApp::ErrorManager::IsQueueNotEmpty(){
    return m_errors.size() != 0;
}

IApp::Error IApp::ErrorManager::GetNextError(){
    IApp::Error error = m_errors.top();
    m_errors.pop();
    return error;
}