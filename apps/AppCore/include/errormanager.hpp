#ifndef IPHYSICS_ERRORMANAGER_HPP
#define IPHYSICS_ERRORMANAGER_HPP
#include <queue>
#include <string>

namespace IApp {

enum class ErrorSeverity { FatalError, NormalError, Warning, Information };

class Error final{
 private:
  ErrorSeverity m_errorSeverity;
  std::string m_errorTitle;
  std::string m_errorMessage;

 public:
  Error(ErrorSeverity errorSeverity, const std::string& errorTitle,
        const std::string& errorMessage);

  ErrorSeverity GetErrorSeverity() const;
  const std::string& GetErrorTitle() const;
  const std::string& GetErrorMessage() const;

  bool operator<(const Error& other) const;
  bool operator==(const Error& other) const;
};

class ErrorManager final{
 private:
  static std::priority_queue<Error> m_errors;

 public:
  static void AddError(Error& error);
  static void AddError(const std::string& title, const std::string& message,
                       ErrorSeverity errorSeverity);
  static bool IsQueueNotEmpty();
  static const Error& GetNextError();
};
}  // namespace IApp

#endif