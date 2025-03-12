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
    attitude_quaternion_valid = false;
    raw_gps_valid = false;
    comp_gps_valid = false;
  }

  void setAsOfMsec(unsigned long asOfMsec)
  {
    this->asOfMsec = asOfMsec;
  }

  unsigned long asOfMsec; // TODO we probably should store an asOf for each element
  bool status_valid;
  bool rc_valid;
  bool attitude_quaternion_valid;
  bool raw_gps_valid;
  bool comp_gps_valid;

  msp_status_t status;
  msp_rc_t rc;
  msp_attitude_quaternion_t attitude_quaternion;
  msp_raw_gps_t raw_gps;
  msp_comp_gps_t comp_gps;

  int autoc_countdown;
  bool autoc_enabled;
  msp_set_raw_rc_t command_buffer;

private:
};

extern State state;
