#pragma once

#include <MSP.h>
#include <vector>
#include <string>

/*
 * MSPSim.h - MSP Simulation Class for XIAO-GP
 *
 * Provides the same interface as MSP class but reads data from
 * generated flight_data.cpp instead of real hardware.
 */

// Forward declaration of FlightDataFrame (defined in generated flight_data.cpp)
struct FlightDataFrame;

class MSPSim {
private:
    uint32_t _timeout;
    uint32_t _start_time_msec;      // When simulation started (millis())
    uint32_t _flight_data_start_us; // First timestamp in flight data
    bool _initialized;
    size_t _current_frame_index;    // Current position in flight data (for O(n) advancement)
    uint32_t _last_target_time_us;  // Last requested time (to detect time going backwards)

    // RC command logging for comparison
    bool _logging_enabled;
    std::vector<std::string> _command_log;

    // Store last GP command for comparison with recorded data
    uint16_t _last_gp_command[4];    // [roll, pitch, yaw, throttle]
    bool _has_gp_command;

    // Sampling pattern detection
    int32_t _sampling_offset_us;     // Offset to apply for better timing correlation
    bool _has_sampling_offset;

    const FlightDataFrame* findFrameAtTime(uint32_t target_time_us);
    bool convertFrameToMSPResponse(uint16_t messageID, const FlightDataFrame* frame,
                                   void* payload, uint16_t maxSize, uint16_t* recvSize);

public:
    void begin(Stream & stream, uint32_t timeout = 500);

    void send(uint16_t messageID, void * payload, uint16_t size);
    bool recv(uint16_t *messageID, void * payload, uint16_t maxSize, uint16_t *recvSize);

    bool waitFor(uint16_t messageID, void * payload, uint16_t maxSize, uint16_t *recvSize = NULL);

    bool request(uint16_t messageID, void * payload, uint16_t maxSize, uint16_t *recvSize = NULL);

    // Overloaded request for waypoint requests (with request payload)
    bool request(uint16_t messageID, void * requestPayload, uint16_t requestSize,
                 void * responsePayload, uint16_t maxResponseSize, uint16_t *recvSize = NULL);

    bool command(uint16_t messageID, void * payload, uint16_t size, bool waitACK = true);

    void reset();

    bool getActiveModes(uint32_t * activeModes);

    // Simulation-specific methods
    void enableCommandLogging(bool enable = true);
    void getCommandLog(std::vector<std::string>& log);
    void clearCommandLog();
    size_t getCurrentFrameIndex() const;

    // Command correlation analysis
    int findCommandCorrelation(const uint16_t gp_cmd[3], int max_frames_ahead = 10, float tolerance = 50.0f);
    bool getFrameCommands(size_t frame_index, uint16_t cmd[3]);

    // Sampling pattern detection and timing offset
    void analyzeSamplingPattern();
    int32_t getSamplingOffset() const;
    bool hasSamplingOffset() const;
};