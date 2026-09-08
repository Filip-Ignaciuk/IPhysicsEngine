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
        Error(ErrorSeverity errorSeverity, const std::string& errorTitle, const std::string& errorMessage);

        ErrorSeverity GetErrorSeverity() const;
        std::string GetErrorTitle() const;
        std::string GetErrorMessage() const;

        bool operator<(const Error& other) const;
        bool operator==(const Error& other) const;
    };

    class ErrorManager{
        private:
        static std::priority_queue<Error> m_errors;
        
        public:
        static void AddError(Error& error);
        static void AddError(const std::string& title, const std::string& message, ErrorSeverity errorSeverity);
        static bool IsQueueNotEmpty();
        static Error GetNextError();



    };
}