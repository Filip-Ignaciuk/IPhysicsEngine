#include "errormanager.hpp"

IApp::Error::Error(ErrorSeverity errorSeverity, const std::string& errorTitle,
                   const std::string& errorMessage)
    : m_errorSeverity(errorSeverity),
      m_errorTitle(errorTitle),
      m_errorMessage(errorMessage) {}

IApp::ErrorSeverity IApp::Error::GetErrorSeverity() const {
  return m_errorSeverity;
}

const std::string& IApp::Error::GetErrorTitle() const { return m_errorTitle; }

const std::string& IApp::Error::GetErrorMessage() const { return m_errorMessage; }

bool IApp::Error::operator<(const Error& other) const {
  return this->m_errorSeverity < other.m_errorSeverity;
}

bool IApp::Error::operator==(const Error& other) const {
  return this->m_errorSeverity == other.m_errorSeverity &&
         this->m_errorTitle.compare(other.m_errorTitle) == 0 &&
         this->m_errorMessage.compare(other.m_errorMessage) == 0;
}

std::priority_queue<IApp::Error> IApp::ErrorManager::m_errors;

void IApp::ErrorManager::AddError(Error& error) { m_errors.emplace(error); }

void IApp::ErrorManager::AddError(const std::string& title,
                                  const std::string& message,
                                  ErrorSeverity errorSeverity) {
  Error error(errorSeverity, title, message);
  m_errors.emplace(error);
}

bool IApp::ErrorManager::IsQueueNotEmpty() { return m_errors.size() != 0; }

const IApp::Error& IApp::ErrorManager::GetNextError() {
  const IApp::Error& error = m_errors.top();
  m_errors.pop();
  return error;
}