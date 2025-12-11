#include <thread>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <cassert>

#include "gyro.h"

void GyroAPI::startUpdateLoop(char *folder_name)
{
  m_run_thread = true;
  for (unsigned int i = 0; i < m_devices.size(); i++)
  {
    std::filesystem::path log_file_path = std::filesystem::path(folder_name) / std::filesystem::path("sensor" + std::to_string(i) + ".csv");
    // std::filesystem::path log_file_path = std::filesystem::path("sensor" + std::to_string(i) + ".csv");
    std::ofstream *file_stream = new std::ofstream();
    file_stream->open(log_file_path);
    m_file_streams.emplace_back(file_stream);
    *m_file_streams.back() << "time (us),x(mdps),y(mdps),z(mdps)\n";

    m_last_times.push_back(0);
  }
  m_thread = std::thread(std::bind(&GyroAPI::gyro_thread, this));
}

void GyroAPI::setRecord(bool value, int frequency)
{
  m_record = value;
  m_frequency = frequency;
}

bool GyroAPI::checkRegister(uint8_t address, uint8_t reg, uint8_t expected)
{

  m_wire.beginTransmission(address);
  m_wire.write(reg);
  if (m_wire.endTransmission() == -1)
  {
    std::cout << "[GRYO] Check register failed for register 0x" << std::hex << (int)reg << std::endl;
    return false;
  }
  m_wire.requestFrom(address, 1);
  uint8_t read = m_wire.read();
  std::cout << "Checking register...\n";
  std::cout << "-- reg 0x" << std::hex << (int)reg
            << "\n-- read: 0x" << (int)read
            << "\n-- expected: 0x" << (int)expected << std::endl;

  if (!(read == expected))
  {
    return false;
  }

  return true;
}

void GyroAPI::add_device(uint8_t address)
{
  SparkFun_ISM330DHCX *new_device = new SparkFun_ISM330DHCX();
  if (!new_device->begin(m_wire, address))
    std::cout << "[ERROR] SparkFun_ISM330DHCX init() failed.\n";

  if (!new_device->deviceReset())
  {
    std::cout << "[GYRO] Failed to reset device.\n";
    delete new_device;
    return;
  }

  // Confirm communication is established 
  uint8_t who_am_i = new_device->getUniqueId();
  if (who_am_i != 0x6b)
  {
    std::cout << "WHO_AM_I failed.\n";
    delete new_device;
    return;
  }

  if (!new_device->setDeviceConfig())
  {
    std::cout << "[GYRO] setDeviceConfig failed.\n";
    delete new_device;
    return;
  } // 0x18 R/W
  checkRegister(address, 0x18, 0xe2);

  if (!new_device->setBlockDataUpdate())
  {
    std::cout << "[GYRO] setBlockDataUpdate failed.\n";
    delete new_device;
    return;
  } // 0x12 R/W
  if (!new_device->setGyroDataRate(ISM_GY_ODR_3332Hz))
  {
    std::cout << "[GYRO] setGyroDataRate failed.\n";
    delete new_device;
    return;
  } //
  if (!new_device->setGyroFullScale(ISM_250dps))
  {
    std::cout << "[GYRO] setGyroFullScale failed.\n";
    delete new_device;
    return;
  } // 0x11 R/W
  checkRegister(address, 0x11, (ISM_GY_ODR_3332Hz << 4) | ISM_250dps);

  if (!new_device->setGyroFilterLP1())
  {
    std::cout << "[GYRO] setGyroFilterLP1 failed.\n";
    delete new_device;
    return;
  } // 0x13 R/W
  if (!new_device->setGyroLP1Bandwidth(ISM_MEDIUM))
  {
    std::cout << "[GYRO] setGyroLP1Bandwidth failed.\n";
    delete new_device;
    return;
  } // 0x15 R/W
  checkRegister(address, 0x15, ISM_MEDIUM);

  /// TODO: Set up accelerometer 

  // If successful
  m_devices.push_back(new_device);
  std::cout << "\tAdded device with address 0x" << std::hex << (int)address << std::dec << std::endl;
  std::cout << "\t\tDevice will log to sensor" << m_devices.size() - 1 << ".csv" << std::endl;
}

void GyroAPI::flush()
{
  for (auto &stream : m_file_streams)
  {
    stream->flush();
  }
}

void GyroAPI::join()
{
  if (m_thread.joinable())
    m_thread.join();

  // for (auto &device : m_devices)
  // {
  //   device->deviceReset(); 
  //   delete device; 
  // }
  // m_wire.end(); 
}

bool GyroAPI::statusCheck()
{
  if (m_devices.size() == 0)
  {
    std::cout << "No ISM330DHCX devices detected. Please check connections." << std::endl;
    return false;
  }
  // Check connection status of all devices
  for (auto &device : m_devices)
    if (!device->isConnected())
      return false;
  return true;
}
void GyroAPI::stopUpdateLoop()
{
  m_run_thread = false;
  join();
  flush();
  for (auto &stream : m_file_streams)
  {
    stream->close();
  }
}





void GyroAPI::gyro_thread()
{


  while (m_run_thread)
  {
    for (int index = 0; index < m_devices.size(); index++)
    {
      if (!m_run_thread)
        break;

      // Get current time
      int64_t now_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

      // Save results to file
      if ((1000000000.0 / (now_time - m_last_times[index]) <= m_frequency) && m_record)
      {
        double current_rate = 1000000000.0 / (now_time - m_last_times[index]);
        std::cout << "\rCurrent rate: " << current_rate << " Hz     " << std::flush;
        m_last_times[index] = now_time;

        // Test: bypass status check and try to read gyro data directly
        if (m_devices[index]->checkGyroStatus())
        {
          sfe_ism_data_t gyroData;
          m_devices[index]->getGyro(&gyroData);
          now_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

          // std::cout << "[GYRO] Data: X=" << gyroData.xData << " Y=" << gyroData.yData << " Z=" << gyroData.zData << std::endl;

          *m_file_streams[index] << now_time
                                 << "," << gyroData.xData
                                 << "," << gyroData.yData
                                 << "," << gyroData.zData << "," << std::endl;

          // TODO: Log accelerometer if wanted 
          // sfe_ism_data_t accelData;
          // m_devices[index]->getAccel(&accelData);
          
          // now_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
          // *m_file_streams[index] << now_time
          //                        << "," << accelData.xData
          //                        << "," << accelData.yData
          //                        << "," << accelData.zData << "," << std::endl;
        }
        else
        {
          std::cout << "[GYRO] Status check: Gyro data not ready for device " << index << ". Data will not be logged.\n";
        }
      }
    }
  }
  std::cout << "Gyro thread stopped." << std::endl;
  return;
}
