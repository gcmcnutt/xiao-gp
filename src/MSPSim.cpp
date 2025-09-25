/*
 * MSPSim.cpp - MSP Simulation Implementation for XIAO-GP
 *
 * Implements the same MSP interface as MSP.cpp but fetches data from
 * generated flight_data.cpp instead of real hardware. This allows
 * replay of recorded flight scenarios for GP testing and validation.
 *
 * The timestamp offset logic handles the difference between xiao's millis()
 * timer and the absolute timestamps in the flight data.
 */

#include <MSPSim.h>
#include <main.h> // For logPrint
#include <math.h> // For sqrt

// MSP message IDs for debug logging
#define MSP_STATUS 101
#define MSP_RC 105
#define MSP_NAV_STATUS 121

// Flight data frame structure (must match generated flight_data.cpp)
struct FlightDataFrame {
    uint32_t time_us;
    msp_status_t status;
    msp_attitude_quaternion_t attitude_quaternion;
    msp_waypoint_t waypoint;
    uint16_t servo[3];          // servo outputs: aileron, elevator, throttle (AET)
};

// External declarations for generated flight data functions
extern const FlightDataFrame* get_flight_data_frame(size_t index);
extern size_t get_flight_data_frame_count();
extern const FlightDataFrame* get_flight_data_frame_at_time_msec(uint32_t time_msec);

void MSPSim::begin(Stream & stream, uint32_t timeout) {
        _timeout = timeout;
        _initialized = false;
        _current_frame_index = 0;
        _last_target_time_us = 0;
        _logging_enabled = false;
        _command_log.clear();
        _has_gp_command = false;

        // Get the first timestamp from flight data as our reference
        if (get_flight_data_frame_count() > 0) {
            const FlightDataFrame* first_frame = get_flight_data_frame(0);
            if (first_frame) {
                _flight_data_start_us = first_frame->time_us;
                _start_time_msec = millis();
                _last_target_time_us = first_frame->time_us;
                _initialized = true;

                logPrint(INFO, "MSPSim: Initialized with %u frames, start_time=%lu us (%lu ms)",
                         (unsigned int)get_flight_data_frame_count(), _flight_data_start_us, _start_time_msec);
            }
        }
    }

void MSPSim::send(uint16_t messageID, void * payload, uint16_t size) {
        // Keep existing command log for backwards compatibility if enabled
        if (_logging_enabled && messageID == MSP_SET_RAW_RC && payload && size >= sizeof(msp_rc_t)) {
            const msp_rc_t* rc = static_cast<const msp_rc_t*>(payload);
            uint32_t elapsed_time_msec = millis() - _start_time_msec;

            char log_entry[128];
            snprintf(log_entry, sizeof(log_entry),
                "T:%lu RC:[%u,%u,%u,%u,%u,%u,%u,%u]",
                (unsigned long)elapsed_time_msec,
                rc->channelValue[0], rc->channelValue[1], rc->channelValue[2], rc->channelValue[3],
                rc->channelValue[4], rc->channelValue[5], rc->channelValue[6], rc->channelValue[7]);

            _command_log.push_back(std::string(log_entry));
        }
    }

bool MSPSim::recv(uint16_t *messageID, void * payload, uint16_t maxSize, uint16_t *recvSize) {
        // Not typically used directly in msplink.cpp
        return false;
    }

bool MSPSim::waitFor(uint16_t messageID, void * payload, uint16_t maxSize, uint16_t *recvSize) {
        // Delegate to request()
        return request(messageID, payload, maxSize, recvSize);
    }

// Main request method used by msplink.cpp
bool MSPSim::request(uint16_t messageID, void * payload, uint16_t maxSize, uint16_t *recvSize) {
        if (!_initialized) {
            return false;
        }

        // Calculate elapsed time since simulation started
        uint32_t elapsed_sim_time_msec = millis() - _start_time_msec;

        // Calculate target timestamp in flight data
        // We add elapsed time to the first frame's timestamp
        uint32_t target_time_us = _flight_data_start_us + (elapsed_sim_time_msec * 1000);

        // Find frame at or after target time
        const FlightDataFrame* frame = findFrameAtTime(target_time_us);
        if (!frame) {
            logPrint(INFO, "MSPSim: No frame found for target_time=%lu us, elapsed=%lu ms", target_time_us, elapsed_sim_time_msec);
            return false; // No data available at this time
        }

        // Debug logging for MSP request details
        if (messageID == MSP_STATUS) {
            bool hasARM = (frame->status.flightModeFlags & 1);
            bool hasMSPRC = (frame->status.flightModeFlags & (1UL << 30));
            logPrint(INFO, "MSPSim: MSG=STATUS target_time=%lu us, frame_idx=%u, frame_time=%lu us, flags=0x%08lX (ARM=%s, MSPRC=%s) recorded_cmd=[%u,%u,%u]",
                     target_time_us, (unsigned int)_current_frame_index, frame->time_us, (unsigned long)frame->status.flightModeFlags,
                     hasARM ? "Y" : "N", hasMSPRC ? "Y" : "N",
                     frame->servo[0], frame->servo[1], frame->servo[2]);
        }

        // Convert frame data to MSP response based on message ID
        return convertFrameToMSPResponse(messageID, frame, payload, maxSize, recvSize);
    }

// Overloaded request for waypoint requests (with request payload)
bool MSPSim::request(uint16_t messageID, void * requestPayload, uint16_t requestSize,
                     void * responsePayload, uint16_t maxResponseSize, uint16_t *recvSize) {

        // For MSP_WP requests, we ignore the request payload and just return current position
        return request(messageID, responsePayload, maxResponseSize, recvSize);
    }

bool MSPSim::command(uint16_t messageID, void * payload, uint16_t size, bool waitACK) {
        send(messageID, payload, size);
        return true; // Assume commands always succeed in simulation
    }

void MSPSim::reset() {
        // Reset simulation timing and index tracking
        _current_frame_index = 0;
        _last_target_time_us = 0;
        _command_log.clear();

        if (_initialized && get_flight_data_frame_count() > 0) {
            const FlightDataFrame* first_frame = get_flight_data_frame(0);
            if (first_frame) {
                uint32_t old_start = _flight_data_start_us;
                uint32_t old_time = _start_time_msec;
                _flight_data_start_us = first_frame->time_us;
                _start_time_msec = millis();
                _last_target_time_us = first_frame->time_us;

                logPrint(INFO, "MSPSim: Reset timing - old_start=%lu us (%lu ms), new_start=%lu us (%lu ms)",
                         old_start, old_time, _flight_data_start_us, _start_time_msec);
            }
        }
    }

bool MSPSim::getActiveModes(uint32_t * activeModes) {
        if (!_initialized) {
            return false;
        }

        uint32_t elapsed_sim_time_msec = millis() - _start_time_msec;
        uint32_t target_time_us = _flight_data_start_us + (elapsed_sim_time_msec * 1000);
        const FlightDataFrame* frame = findFrameAtTime(target_time_us);

        if (frame) {
            *activeModes = frame->status.flightModeFlags;
            return true;
        }
        return false;
    }

const FlightDataFrame* MSPSim::findFrameAtTime(uint32_t target_time_us) {
        // Check if time went backwards (simulation reset or time jump)
        if (target_time_us < _last_target_time_us) {
            _current_frame_index = 0;
        }

        // If requesting same timestamp as last call, return the same frame without advancing index
        if (target_time_us == _last_target_time_us && _current_frame_index < get_flight_data_frame_count()) {
            const FlightDataFrame* frame = get_flight_data_frame(_current_frame_index);
            if (frame && frame->time_us >= target_time_us) {
                return frame; // Return same frame for same timestamp
            }
        }

        // Time moved forward - search from current position
        size_t frame_count = get_flight_data_frame_count();
        for (size_t i = _current_frame_index; i < frame_count; i++) {
            const FlightDataFrame* frame = get_flight_data_frame(i);
            if (frame && frame->time_us >= target_time_us) {
                _current_frame_index = i; // Remember position for next call
                _last_target_time_us = target_time_us;
                return frame;
            }
        }

        // If we reach here, no frame found at or after target time
        _current_frame_index = frame_count; // Set to end to avoid re-scanning
        _last_target_time_us = target_time_us;
        return nullptr;
    }

bool MSPSim::convertFrameToMSPResponse(uint16_t messageID, const FlightDataFrame* frame,
                                        void* payload, uint16_t maxSize, uint16_t* recvSize) {

        switch (messageID) {
            case MSP_STATUS:
                if (maxSize >= sizeof(msp_status_t)) {
                    memcpy(payload, &frame->status, sizeof(msp_status_t));
                    if (recvSize) *recvSize = sizeof(msp_status_t);
                    return true;
                }
                break;


            case MSP_ATTITUDE_QUATERNION:
                if (maxSize >= sizeof(msp_attitude_quaternion_t)) {
                    memcpy(payload, &frame->attitude_quaternion, sizeof(msp_attitude_quaternion_t));
                    if (recvSize) *recvSize = sizeof(msp_attitude_quaternion_t);
                    return true;
                }
                break;

            case MSP_WP:
                if (maxSize >= sizeof(msp_waypoint_t)) {
                    memcpy(payload, &frame->waypoint, sizeof(msp_waypoint_t));
                    if (recvSize) *recvSize = sizeof(msp_waypoint_t);
                    return true;
                }
                break;


            case MSP_NAV_STATUS:
                if (maxSize >= sizeof(msp_nav_status_t)) {
                    msp_nav_status_t nav_status = {};
                    // Fill with defaults for now - navVel data accessed via getNavVel()
                    nav_status.mode = 0;
                    nav_status.state = 0;
                    nav_status.activeWpAction = 0;
                    nav_status.activeWpNumber = 0;
                    nav_status.error = 0;
                    nav_status.magHoldHeading = 0;

                    memcpy(payload, &nav_status, sizeof(msp_nav_status_t));
                    if (recvSize) *recvSize = sizeof(msp_nav_status_t);
                    return true;
                }
                break;

            default:
                // Unsupported message type
                return false;
        }

        return false; // Buffer too small or other error
}

// Simulation-specific methods for command logging and diagnostics
void MSPSim::enableCommandLogging(bool enable) {
    _logging_enabled = enable;
    if (!enable) {
        _command_log.clear();
    }
}

void MSPSim::getCommandLog(std::vector<std::string>& log) {
    log = _command_log;
}

void MSPSim::clearCommandLog() {
    _command_log.clear();
}

size_t MSPSim::getCurrentFrameIndex() const {
    return _current_frame_index;
}

