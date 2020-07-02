#include "lepton_thermal_sensor/sensor_handler.h"

SensorHandler::SensorHandler(std::string i2c_dev, std::string spi_dev, uint32_t spi_speed, uint16_t packets_per_frame, uint16_t packet_size, uint8_t head_bytes, uint16_t reboot_max_cnt):
  i2c_dev_(i2c_dev), spi_dev_(spi_dev), spi_speed_(spi_speed),
  packets_per_frame_(packets_per_frame), packet_size_(packet_size), head_bytes_(head_bytes),
  connected_(false), wrong_packet_cnt_(0)
{
  raw_data_ = new uint8_t[packets_per_frame_ * packet_size_];
}

SensorHandler::~SensorHandler()
{
  delete raw_data_;
}

const float SensorHandler::getSensorTemperature()
{
  /* get reference temperature at focal array in x100 Kelvin */
  int ref_temp = Lepton::I2C::fpaTemperature();

  return ref_temp/100.0;
}

void SensorHandler::connect()
{
  if(!Lepton::I2C::openPort(i2c_dev_))
    {
      ROS_FATAL_STREAM("cannot open I2C port: " << i2c_dev_ );
      return;
    }

  if(!Lepton::SPI::openPort(spi_dev_, spi_speed_))
    {
      ROS_FATAL_STREAM("cannot open SPI port ");
      return;
    }

  connected_ = true;
}

void SensorHandler::disconnect()
{
  Lepton::SPI::closePort();
  Lepton::I2C::closePort();

  connected_ = false;
}

void SensorHandler::reboot()
{
  Lepton::SPI::closePort();
  Lepton::I2C::reboot();
  usleep(750000);
  Lepton::SPI::openPort(spi_dev_, spi_speed_);
  sleep(3); // wait for 3 seconds
}

bool SensorHandler::grabImage(cv::Mat &temp_map, double& timestamp)
{
  if(!connected_) return false;

  timestamp = ros::Time::now().toSec();

  for (int j = 0; j < packets_per_frame_; j++)
    {
      Lepton::SPI::read(raw_data_ + sizeof(uint8_t) * packet_size_ * j, sizeof(uint8_t) * packet_size_);
      int row = raw_data_[j * packet_size_ + 1];

      /* wrong packet: reset */
      if (row != j)
        {
          j = -1;
          wrong_packet_cnt_ += 1;
          usleep(1000);

          if (wrong_packet_cnt_ == reboot_max_cnt_) reboot();

          timestamp = ros::Time::now().toSec();
        }
    }

  double start_t = ros::Time::now().toSec();
  for (int j = 0; j < packets_per_frame_; j++)
    {
      /* straightfoward swapping: ~0.002 sec */
      /*
      for(int k = 0; k < packet_size_ / 2; k++)
        {
        uint8_t temp = raw_data_.at(packet_size_ * j + k * 2);
          raw_data_.at(packet_size_ * j + k * 2) = raw_data_.at(packet_size_ * j + k * 2 + 1);
          raw_data_.at(packet_size_ * j + k * 2 + 1) = temp;
        }
      */
      memcpy(temp_map.ptr(j), raw_data_ + packet_size_ * j + head_bytes_, packet_size_ - head_bytes_);
    }

  /* parallel swapping: ~0.0005 sec */
  temp_map.forEach<uint16_t>([&](uint16_t &p, const int position[2]) -> void
    {
      uint8_t temp = *(uint8_t*)(&p);
      *(uint8_t*)(&p) = *((uint8_t*)(&p) + 1);
      *((uint8_t*)(&p) + 1) = temp;
    });

  // ROS_INFO("data mapping time: %f", ros::Time::now().toSec() - start_t); //debug

  return true;
}
