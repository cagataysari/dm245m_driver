#pragma once

#include <iostream>
#include <modbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <ros/ros.h>

#define PARAM_CURRENT 0x00
#define PARAM_LOCKING_CURRENT 0x01
#define PARAM_MICRO_STEPS 0x02

#define PARAM_PULSE_INPUT_MODE 0x0004
#define PULSE_MODE_POSITIVE_NEGATIVE_TRAILING 0x0000
#define PULSE_MODE_PULSE_DIRECTION_TRAILING 0x0001
#define PULSE_MODE_POSITIVE_NEGATIVE_RAISING 0x0002
#define PULSE_MODE_PULSE_DIRECTION_RAISING 0x0003

#define PARAM_INITIAL_SPEED 0x0006

#define PARAM_ACC_SPEED 0x0007

#define PARAM_MAX_SPEED 0x0008

#define PARAM_PULSE_COUNT_HIGH 0x09
#define PARAM_PULSE_COUNT_LOW 0x0A

#define PARAM_CHECK_STATUS 0x23
#define CHECK_STATUS_WORKING 0x55
#define CHECK_STATUS_STOPPED 0xAA

#define PARAM_START_STOP 0x50
#define START_POSITIVE 0x55
#define START_NEGATIVE 0x155
#define STOP_GRADUALLY 0xAA
#define STOP_IMMEDIATELY 0xCC

#define P_ERROR -1
#define P_SUCCESS 0

#define BUFFER_LEN 20


class DM245MDriver{

private:
  //! controller pointer
  modbus_t* ctx_;
  //! communication buffer
  std::unique_ptr<uint16_t> buffer_;
  //// Parameters of DM245M ////
  std::string device_name_;
  //! Connection Baud Rate
  unsigned int baud_rate_;
  //! slave number of dm245
  unsigned char port_no_;
  //! connection timeout
  unsigned char time_out_;
  //! in debug mode ?
  bool debug_mode_;
  //! Max Current
  uint16_t max_current_;
  //! Locking Current
  uint16_t locking_current_;
  //! High side pulse
  uint16_t high_side_pulse_;
  //! Low side pulse
  uint16_t low_side_pulse_;
  //! Direction
  bool direction_;
  //! logger zone
  const std::string log_zone_;

public:

  //!
  //! \brief DM245MDriver: constructor with params
  //! \param arguments: list of arguments
  //!
  DM245MDriver(ros::NodeHandle& nh_, ros::NodeHandle& priv_nh_);

  //! Destructor
  ~DM245MDriver();

  //!
  //! \brief check_status: checks status of DM245M
  //! \return status of motor on success, P_ERROR on fail
  //!
  uint8_t check_status(void);

  //!
  //! \brief turn_motor: turns to motor to the given direction with given pulse
  //! \param direction: CCW or CW
  //! \param pulse: number of pulses
  //! \return P_SUCCESS on success, P_ERROR on fail
  //!
  int turn_motor(bool direction, uint32_t pulse);

  //!
  //! \brief stop_motor: stops motor
  //! \param instant: instanly or gradually ?
  //! \return P_SUCCESS on success, P_ERROR on fail
  //!
  int stop_motor(bool instant);

  //!
  //! \brief close_driver: resets serial communication and dumps controller pointer from memory
  //! \return
  //!
  void close_driver(void);

  //!
  //! \brief get_max_current: get maximum current
  //! \return max current of the driver in mA, P_ERROR on fail
  //!
  int get_max_current();

  //!
  //! \brief set_max_current: sets maximum current
  //! \param max_c_ maximum current in mA
  //! \return P_SUCCESS on success, P_ERROR on fail
  //!
  int set_max_current(uint8_t max_c_);

  //!
  //! \brief get_locking_current: get locking current
  //! \return locking current on success, P_ERROR on fail
  //!
  uint8_t get_locking_current();

  //!
  //! \brief set_locking_current: sets locking current
  //! \return P_SUCCESS on success, P_ERROR on fail
  //!
  int set_locking_current(uint8_t locking_current_);

private:
  //!
  //! \brief init: initialises motor driver
  //!
  void init(void);
  //!
  //! \brief init_dm245m: initialises the drivers
  //! \return P_SUCCESS on success, P_ERROR on fail
  //!
  int init_dm245m(void);

  //!
  //! \brief set_param: sets the parameters in the address
  //! \param addr: Address of the parameters
  //! \param val: Value off the parameter
  //! \return P_SUCCESS on success, P_ERROR on fail
  //!
  int set_param(int addr, int val);

  //!
  //! \brief get_param: gets the parameters in the address
  //! \param addr: Address of the parameters
  //! \param no_of_bytes: bytes returned from register
  //! \return P_SUCCESS on success, P_ERROR on fail
  //!
  int get_param(int addr, int no_of_bytes);

};

