#include <dm245m_driver/DM245M.h>
#include <string>
#include <modbus.h>


DM245MDriver::DM245MDriver(ros::NodeHandle& nh_, ros::NodeHandle& priv_nh_):
  log_zone_("[ SKRobotik.DM245MDriver ] "),
  device_name_("/dev/ttyUSB0"),
  baud_rate_(115200),
  port_no_(1),
  time_out_(2),
  debug_mode_(true),
  buffer_(new uint16_t[BUFFER_LEN]),
  ctx_(nullptr),
  max_current_(2800),
  direction_(false),
  high_side_pulse_(0),
  low_side_pulse_(0),
  locking_current_(50)
{
  ROS_INFO_STREAM(this->log_zone_ << "Initalising DM245M with default arguments");
  this->init();
}

DM245MDriver::~DM245MDriver()
{
  this->close_driver();
}

void DM245MDriver::init()
{
  for(int i = 0 ; i < BUFFER_LEN ; i++) this->buffer_.get()[i] = 0;
  if(this->init_dm245m() == P_ERROR)
  {
    this->close_driver();
    this->buffer_.release();
    throw "FAIL";
  }
}

int DM245MDriver::init_dm245m(void)
{
  this->ctx_ = modbus_new_rtu(device_name_.c_str(), baud_rate_, 'N', 8 ,1);
  if ( this->ctx_ == nullptr)
  {
    ROS_FATAL_STREAM(this->log_zone_ << "Unable to create modbus context");
    return P_ERROR;
  }
  if(this->debug_mode_)
  {
    ROS_INFO_STREAM(this->log_zone_ << "Debug mode is on");
    modbus_set_debug(this->ctx_, 1);
  }else{
    ROS_INFO_STREAM(this->log_zone_ << "Debug mode is off");
    modbus_set_debug(this->ctx_, 0);
  }


  struct timeval old_response_timeout;
  struct timeval response_timeout;

  modbus_get_response_timeout(this->ctx_, &old_response_timeout);
  response_timeout.tv_sec = this->time_out_;
  response_timeout.tv_usec = 0;
  modbus_set_response_timeout(this->ctx_, &response_timeout);
  
  int slave = modbus_set_slave(this->ctx_,this->port_no_);

  if(slave == -1)
  {
    ROS_ERROR_STREAM(this->log_zone_ << "Unable to connect to slave: " << (int)this->port_no_);
    return P_ERROR;
  }

  int connected_ = modbus_connect(this->ctx_);
  if(connected_ == -1)
  {
    ROS_ERROR_STREAM(this->log_zone_ << "Connection failed");
    modbus_free(this->ctx_);
    return P_ERROR;
  } else if (connected_ == 0)
  {
    ROS_INFO_STREAM(this->log_zone_ << "Connected to DM245, slave no: " << (int)this->port_no_ << " port: " << this->device_name_);
  }
  return P_SUCCESS;
}

int DM245MDriver::get_param(int addr, int no_of_bytes)
{
  int rc=0;
  rc = modbus_read_registers(this->ctx_, addr, 1, this->buffer_.get());
  if( rc == -1)
  {
    ROS_ERROR_STREAM(this->log_zone_ << "Problem with reading: " << modbus_strerror(errno));
    return P_ERROR;
  }else{
    ROS_INFO_STREAM(this->log_zone_ << "Response from DM245M, addr: " << (int)addr << " no_of_bytes: " << (int)rc);
    if(this->debug_mode_)
    {
        for(int i = 0; i < rc; i++)
        {
          ROS_INFO("%s0x%x", this->log_zone_.c_str(), this->buffer_.get()[i]);
        }
    }
  }
  return P_SUCCESS;
}

int DM245MDriver::set_param(int addr, int val)
{
  int rc=0;
  rc = modbus_write_register(this->ctx_, addr, val);
  if(rc == -1)
  {
    ROS_ERROR_STREAM(this->log_zone_ << "Problem with writing: " << modbus_strerror(errno));
    return P_ERROR;
  } else if( rc == 1) {
    ROS_DEBUG_STREAM(this->log_zone_ << "Written addr: " << (int)addr << " val: " << (int)val);
    return P_SUCCESS;
  }
  return P_ERROR;
}

int DM245MDriver::turn_motor(bool direction, uint32_t pulse)
{

  if(pulse > 0)
  {
   uint16_t high_side_ = pulse >> 16;
   uint16_t low_side_ = (uint16_t) pulse;
    if(high_side_ != this->high_side_pulse_)
    {
        if(set_param(PARAM_PULSE_COUNT_HIGH,high_side_) == P_ERROR)
          return P_ERROR;
        else
          ROS_INFO_STREAM(this->log_zone_ << "New high-side pulse written");
        this->high_side_pulse_ = high_side_;
    }
    if(low_side_ != this->low_side_pulse_)
    {
        if(set_param(PARAM_PULSE_COUNT_LOW, low_side_)  == P_ERROR)
           return P_ERROR;
        else
           ROS_INFO_STREAM(this->log_zone_ << "New low-side pulse written");
        this->low_side_pulse_ = low_side_;
    }
  } else {
    ROS_ERROR_STREAM(this->log_zone_ << "Use stop_motor() method to stop");
    return P_ERROR;
  }
  if(direction)
  {
    return set_param(PARAM_START_STOP,START_POSITIVE);
  } else {
    return set_param(PARAM_START_STOP,START_NEGATIVE);
  }
}

int DM245MDriver::stop_motor(bool instant)
{
  if(instant)
    return set_param(PARAM_START_STOP, STOP_GRADUALLY);
  else
    return set_param(PARAM_START_STOP, STOP_IMMEDIATELY);
}

uint8_t DM245MDriver::check_status()
{
  uint8_t status_;
  if(get_param(PARAM_CHECK_STATUS,1) == P_ERROR)
  {
    ROS_ERROR_STREAM(this->log_zone_ << "Error reading status");
    return P_ERROR;
  }
  switch(this->buffer_.get()[0])
  {
      case CHECK_STATUS_WORKING:
           ROS_INFO_STREAM(this->log_zone_ << "Motor is turning");
           status_ = CHECK_STATUS_WORKING;
      break;
      case CHECK_STATUS_STOPPED:
           ROS_INFO_STREAM(this->log_zone_ << "Motor stops");
           status_ = CHECK_STATUS_STOPPED;
      break;
      default:
           ROS_WARN_STREAM(this->log_zone_ << "Unknown status of motor");
           return P_ERROR;
  }
  return status_;
}

int DM245MDriver::get_max_current()
{
    uint16_t current_;
    if(get_param(PARAM_CURRENT,1) == P_ERROR)
    {
      ROS_ERROR_STREAM(this->log_zone_ << "Error reading maximum current");
      return P_ERROR;
    }
    current_ = this->buffer_.get()[0];
    current_ *= 100;
    if(max_current_ != current_)
       max_current_ = current_;
    return (current_);
}

int DM245MDriver::set_max_current(uint8_t max_c_)
{
  if(max_c_ > 40)
  {
    ROS_WARN_STREAM(this->log_zone_ << "Max current is over 4000mA, decreasing value");
    max_c_ = 40;
  }
  return set_param(PARAM_CURRENT, max_c_);
}

uint8_t DM245MDriver::get_locking_current()
{
    uint8_t locking_curr_;
    if(get_param(PARAM_LOCKING_CURRENT,1) == P_ERROR)
    {
      ROS_ERROR_STREAM(this->log_zone_ << "Unable to get locking current");
      return P_ERROR;
    }
    ROS_INFO_STREAM(this->log_zone_ << "Locking current is: " << (int)this->buffer_.get()[0]);
    locking_curr_ = static_cast<uint8_t>(this->buffer_.get()[0]);
    return locking_curr_;
}

int DM245MDriver::set_locking_current(uint8_t locking_current_)
{
  if(locking_current_ > 100)
  {
    ROS_WARN_STREAM(this->log_zone_ << "Locking current exceeds 100, decreasing it to 100");
    locking_current_ = 100;
  }
  return set_param(PARAM_LOCKING_CURRENT, locking_current_);
}

void DM245MDriver::close_driver()
{
  if(this->ctx_ != nullptr)
  {
    ROS_INFO_STREAM(this->log_zone_ << "Shutting DM245M driver down");
    modbus_close(this->ctx_);
    modbus_free(this->ctx_);
  } else {
    ROS_WARN_STREAM(this->log_zone_ << "DM245M driver already shutdown");
  }
}
