#pragma once
#include <string>
#include <queue>

namespace IApp{

    enum class ErrorSeverity{
        FatalError,
        NormalError,
        Warning,
        Information
    };

    class Error{
        private:
        ErrorSeverity m_errorSeverity;
        std::string m_errorTitle;
        std::string m_errorMessage;
        public:
        Error(ErrorSeverity _errorSeverity, const std::string& _errorTitle, const std::string& _errorMessage);

        ErrorSeverity GetErrorSeverity() const;
        std::string GetErrorTitle() const;
        std::string GetErrorMessage() const;

        bool operator<(const Error& _other) const;
        bool operator==(const Error& _other) const;
    };

    class ErrorManager{
        private:
        static std::priority_queue<Error> m_errors;
        
        public:
        static void AddError(Error& _error);
        static void AddError(const std::string& _title, const std::string& _message, ErrorSeverity _errorSeverity);
        static bool IsQueueNotEmpty();
        static Error GetNextError();



    };
}