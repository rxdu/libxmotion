/* 
 * i2c.hpp
 * 
 * Created on: Jan 08, 2018 22:45
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef I2C_HPP
#define I2C_HPP

#include <cstdint>
#include <string>

namespace librav
{
class I2CBus
{
public:
  I2CBus(int bus_id = -1);
  ~I2CBus();

  bool Init(int bus_id = -1);
  bool SetSlaveAddress(int slave_addr, bool force = true);

  bool ReadRegister(int reg_addr, uint8_t *data);
  int32_t ReadBlockData(int reg_addr_base, int data_len, uint8_t *data);
  bool WriteRegister(int reg_addr, int data);

private:
  bool bus_initialized_ = false;
  int bus_id_ = -1;
  int bus_fd_ = -1;
  std::string bus_name_;
  char bus_filename_[20];
  int slave_address_ = -1;
};
}

#endif /* I2C_HPP */
