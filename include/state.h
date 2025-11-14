#include <MSP.h>

/*
 * track and prepare state of device
 *
 * a state is a set of values from Inav -- status, rc, attitude, gps, etc.
 * the state is updated periodically by querying Inav via MSP
 */
class State
{
public:
  void resetState()
  {
    status_valid = false;
    rc_valid = false;
    altitude_valid = false;
    waypoint_valid = false;
    local_state_valid = false;
    inavSampleTimeMsec = 0;
  }

  void setAsOfMsec(unsigned long asOfMsec)
  {
    this->asOfMsec = asOfMsec;
  }

  unsigned long asOfMsec; // TODO we probably should store an asOf for each element
  unsigned long inavSampleTimeMsec; // INAV's timestamp in msec (from timestamp_us / 1000)
  bool status_valid;
  bool rc_valid;
  bool altitude_valid;
  bool waypoint_valid;
  bool local_state_valid;

  msp_status_t status;
  msp_rc_t rc;
  msp_altitude_t altitude;
  msp_waypoint_t waypoint;
  msp_local_state_t local_state;

  int autoc_countdown;
  bool autoc_enabled;
  msp_set_raw_rc_t command_buffer;

private:
};

extern State state;
