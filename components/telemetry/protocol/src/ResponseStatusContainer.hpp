#pragma once

/**
 * @brief Outcome of a transmitter operation
 */
enum TransmitterStatus : int
{
    TRANSMITTER_OK = 0,    
    TRANSMITTER_ERROR = 1,
};

/**
 * @brief Class that contains the response status and the response message
 */
class ResponseStatusContainer
{
    private:
        int code;
        String description;
    public:
        ResponseStatusContainer(int code, String description) : code(code), description(description) {};
        int getCode() const { return code; }
        String getDescription() const { return description; }

        bool isOk() const { return code == TRANSMITTER_OK; }

        static ResponseStatusContainer ok(String description) { return {TRANSMITTER_OK, description}; }
        static ResponseStatusContainer error(String description) { return {TRANSMITTER_ERROR, description}; }
};