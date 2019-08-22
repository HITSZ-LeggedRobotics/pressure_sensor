#ifndef PressureSensorRS485Driver_H
#define PressureSensorRS485Driver_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
# include <stdio.h>
# include <stdlib.h>
#include "sim_assiants/FootContacts.h"

using namespace std;
using namespace boost::asio;
namespace sri_driver {

class PressureSensorRS485Driver
{
public:
  PressureSensorRS485Driver(const ros::NodeHandle& node_handle,
                              const string serial_name, const unsigned int buad_rate);
  ~PressureSensorRS485Driver();

  void start();

  bool SerialInitialize();

  void SensorInitialize();

  void SensorReadThread();

  bool DataProscessAndPublishThread();

  friend std::ostream& operator << (std::ostream& out, const boost::asio::streambuf& streambuf);

private:

  std::string toString(const boost::asio::streambuf& ack_buf);
  float ByteToFloat(char* byteArry);

  std::vector<unsigned char> data_frame_01_;
  std::vector<unsigned char> data_frame_02_;
  std::vector<unsigned char> data_frame_03_;
  std::vector<unsigned char> data_frame_04_;

  boost::thread sensor_read_thread_;
  boost::thread data_process_thread_;
  boost::recursive_mutex r_mutex_;

  ros::NodeHandle node_handle_;
  ros::Publisher contact_pub_, force_pub_1, force_pub_2;
  sim_assiants::FootContacts contact_msg;

  string serial_name_;
  unsigned int buad_rate_;
  io_service io_sev_;
  serial_port *sp;
  int initial_pressure_value=0.03;
  std::string set_zero = "AT+ADJZF=1;1;1;1;1;1\r\n";
  std::string set_update_rate = "AT+SMPR=100\r\n";
  std::string set_decouple_matrix = "AT+DCPM=(2497.565,0.000000,0.000000,0.000000,0.000000,0.000000);(0.000000,2530.428,0.000000,0.000000,0.000000,0.000000);(0.000000,0.000000,5176.251,0.000000,0.000000,0.000000);(0.000000,0.000000,0.000000,2494.512,0.000000,0.000000);(0.000000,0.000000,0.000000,0.000000,2493.392,0.000000);(0.000000,0.000000,0.000000,0.000000,0.000000,5101.520)\r\n";
  std::string set_compute_unit = "AT+DCPCU=MVPV\r\n";
  std::string set_recieve_format= "AT+SGDM=(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)\r\n";
  std::string get_data_stream = "AT+GSD\r\n";
  std::string stop_data_stream = "AT+GSD=STOP\r\n";
  unsigned char read_sensor_data_01[8] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};
  unsigned char read_sensor_data_02[8] = {0x02,0x03,0x00,0x00,0x00,0x01,0x84,0x39};
  unsigned char read_sensor_data_03[8] = {0x03,0x03,0x00,0x00,0x00,0x01,0x85,0xE8};
  unsigned char read_sensor_data_04[8] = {0x04,0x03,0x00,0x00,0x00,0x01,0x84,0x5F};
  double update_rate;
  boost::asio::streambuf ack_buf_;
};

}
#endif // PressureSensorRS485Driver_H
