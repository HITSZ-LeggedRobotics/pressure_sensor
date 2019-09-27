#include "pressure_sensor/pressuresensorrs485driver.h"
namespace sri_driver {

PressureSensorRS485Driver::PressureSensorRS485Driver(const ros::NodeHandle& node_handle,
                                                         const string serial_name, const unsigned int buad_rate)
  : node_handle_(node_handle),
    serial_name_(serial_name),
    buad_rate_(buad_rate),
    update_rate(50),
    initial_pressure_value(0.03)
{
  ROS_INFO("Driver Construct");
//  unsigned char frame[8] ={0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};
  if(!node_handle_.getParam("/pressure_sensor/initial_pressure_value", initial_pressure_value))
    {
      ROS_ERROR("Can't find parameter of 'initial_pressure_value'");
//      return false;
    }
  if(!node_handle_.getParam("/pressure_sensor/usb_port", serial_name_))
    {
      ROS_ERROR("Can't find parameter of 'serial_name_'");
//      return false;
    }
  ROS_INFO("Open Serial Port '%s' ",serial_name_.c_str());
  sp = new serial_port(io_sev_);
  if(sp)
    {
      SerialInitialize();
    }

//  force_pub_1 = node_handle_.advertise<geometry_msgs::WrenchStamped>("force_and_torque_ch1", 1);
//  force_pub_2 = node_handle_.advertise<geometry_msgs::WrenchStamped>("force_and_torque_ch2", 1);
  //contact_pub_ = node_handle_.advertise<sim_assiants::FootContacts>("/foot_contacts",1);
  data_frame_01_.resize(7);
  data_frame_02_.resize(7);
  data_frame_03_.resize(7);
  data_frame_04_.resize(7);
}

PressureSensorRS485Driver::~PressureSensorRS485Driver()
{
  write(*sp, boost::asio::buffer(stop_data_stream));
  std::cout<<stop_data_stream<<std::endl;
  if(sp){
      delete sp;
    }
}

void PressureSensorRS485Driver::start()
{

  //SensorInitialize();
  ROS_INFO("Start Sensor ");
//  write(*sp, boost::asio::buffer(get_data_stream));
//  std::cout<<get_data_stream<<std::endl;
  sensor_read_thread_ = boost::thread(boost::bind(&PressureSensorRS485Driver::SensorReadThread, this));
  data_process_thread_ = boost::thread(boost::bind(&PressureSensorRS485Driver::DataProscessAndPublishThread, this));

}


bool PressureSensorRS485Driver::SerialInitialize()
{
  if(!sp){
      return false;
    }
  ROS_INFO("Initailizing Serial Port");
  sp->open(serial_name_);
  sp->set_option(serial_port::baud_rate(buad_rate_));
  sp->set_option(serial_port::flow_control(serial_port::flow_control::none));
  sp->set_option(serial_port::parity(serial_port::parity::none));
  sp->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  sp->set_option(serial_port::character_size(8));
  return true;
}

//void PressureSensorRS485Driver::SensorInitialize()
//{
//  ROS_INFO("Initailizing Sensor");
//  write(*sp, boost::asio::buffer(set_zero));
//  std::cout<<set_zero<<std::endl;
//  read_until(*sp, ack_buf_, "\r\n");
//  std::cout << toString(ack_buf_) <<std::endl;

//  write(*sp, boost::asio::buffer(set_update_rate));
//  std::cout<<set_update_rate<<std::endl;
//  read_until(*sp, ack_buf_, "\r\n");
//  std::cout << toString(ack_buf_) <<std::endl;

//  write(*sp, boost::asio::buffer(set_decouple_matrix));
//  read_until(*sp, ack_buf_, "\r\n");
//  std::cout<<set_decouple_matrix<<std::endl;
//  std::cout << toString(ack_buf_) <<std::endl;

//  write(*sp, boost::asio::buffer(set_compute_unit));
//  read_until(*sp, ack_buf_, "\r\n");
//  std::cout<<set_compute_unit<<std::endl;
//  std::cout << toString(ack_buf_) <<std::endl;

//  write(*sp, boost::asio::buffer(set_recieve_format));
//  read_until(*sp, ack_buf_, "\r\n");
//  std::cout<<set_recieve_format<<std::endl;
//  std::cout << toString(ack_buf_) <<std::endl;


//}

void PressureSensorRS485Driver::SensorReadThread()
{
  ROS_INFO("Get in Sensor Read Thread ");
  ros::Rate rate(update_rate);
  char data_frame_01[7];
  char data_frame_02[7];
  char data_frame_03[7];
  char data_frame_04[7];
  ros::Duration delay(0.1);

  while (ros::ok()) {

      write(*sp, buffer(read_sensor_data_01,8));
      size_t len_01 = read(*sp, buffer(data_frame_01,7));
      for(int i=0;i<len_01;i++){
       data_frame_01_[i] = data_frame_01[i];
              //printf("01:%02X ",data_frame_01_[i]);
      }
      //ROS_INFO("recieve %d bytes in buffer 01: \n", int(len_01));
      write(*sp, buffer(read_sensor_data_02,8));
//      ROS_INFO("write bytes in buffer: \n");
//      delay.sleep();
      size_t len_02 = read(*sp, buffer(data_frame_02,7));
      for(int i=0;i<len_02;i++){
       data_frame_02_[i] = data_frame_02[i];
       //printf("02: %02X ",data_frame_02_[i]);

      }

      write(*sp, buffer(read_sensor_data_03,8));
      size_t len_03 = read(*sp, buffer(data_frame_03,7));
      for(int i=0;i<len_03;i++){
       data_frame_04_[i] = data_frame_03[i];
              //printf("01:%02X ",data_frame_01_[i]);
      }

      write(*sp, buffer(read_sensor_data_04,8));
      size_t len_04 = read(*sp, buffer(data_frame_04,7));
      for(int i=0;i<len_04;i++){
       data_frame_03_[i] = data_frame_04[i];
              //printf("01:%02X ",data_frame_01_[i]);
      }
     // ROS_INFO("recieve %d bytes in buffer 02: \n", int(len_02));
      boost::recursive_mutex::scoped_lock lock(r_mutex_);
//      for(int i=0;i<len_01;i++){
//       data_frame_01_[i] = data_frame_01[i];

////       printf(" %02X ",data_frame_01_[i]);
//      }

      lock.unlock();
//      int pressure_value = (data_frame[3]<<8)|data_frame[4];
//      std::cout<<"Pressure : "<<pressure_value<<std::endl;
//      boost::recursive_mutex::scoped_lock unlock(r_mutex_);
      rate.sleep();
    }
//  write(*sp, boost::asio::buffer(stop_data_stream));
  std::cout<<stop_data_stream<<std::endl;
}


bool PressureSensorRS485Driver::DataProscessAndPublishThread()
{
  ROS_INFO("Get in Data Process and Publish Thread 01 ");
  ros::Rate rate(100);
  initial_bias.resize(4);
  contact_pub_ = node_handle_.advertise<sim_assiants::FootContacts>("/foot_contacts",1);
  ros::Duration delay(1);
  delay.sleep();
  initial_bias[0] = (float)((data_frame_01_[3]<<8)|data_frame_01_[4])/2000.0;
  initial_bias[1] = (float)((data_frame_02_[3]<<8)|data_frame_02_[4])/2000.0;
  initial_bias[2] = (float)((data_frame_03_[3]<<8)|data_frame_03_[4])/2000.0;
  initial_bias[3] = (float)((data_frame_04_[3]<<8)|data_frame_04_[4])/2000.0;
  while (ros::ok()) {
      boost::recursive_mutex::scoped_lock lock(r_mutex_);
      //std::vector<unsigned char> data_frame_copy = data_frame_01_;
      for(int i=0;i<7;i++)
        printf(" %02X",data_frame_01_[i]);

      int pressure_value_01 = (data_frame_01_[3]<<8)|data_frame_01_[4];
      std::cout<<"  Pressure 01 : "<<pressure_value_01<<std::endl;
      float pressure_value_01_ = (float)pressure_value_01/2000;
      std::cout<<"pressure value 01: "<<pressure_value_01_<<" Mpa"<<std::endl;

      for(int i=0;i<7;i++)
        printf(" %02X",data_frame_02_[i]);
      int pressure_value_02 = (data_frame_02_[3]<<8)|data_frame_02_[4];
      std::cout<<"  Pressure 02 : "<<pressure_value_02<<std::endl;
      float pressure_value_02_ = (float)pressure_value_02/2000;
      std::cout<<"pressure value 02: "<<pressure_value_02_<<" Mpa"<<std::endl;

      for(int i=0;i<7;i++)
        printf(" %02X",data_frame_03_[i]);
      int pressure_value_03 = (data_frame_03_[3]<<8)|data_frame_03_[4];
      std::cout<<"  Pressure 03 : "<<pressure_value_03<<std::endl;
      float pressure_value_03_ = (float)pressure_value_03/2000;
      std::cout<<"pressure value 03: "<<pressure_value_03_<<" Mpa"<<std::endl;

      for(int i=0;i<7;i++)
        printf(" %02X",data_frame_04_[i]);
      int pressure_value_04 = (data_frame_04_[3]<<8)|data_frame_04_[4];
      std::cout<<"  Pressure 04 : "<<pressure_value_04<<std::endl;
      float pressure_value_04_ = (float)pressure_value_04/2000;
      std::cout<<"pressure value 04: "<<pressure_value_04_<<" Mpa"<<std::endl;
      lock.unlock();
      contact_msg.foot_contacts.resize(4);
      contact_msg.foot_contacts[0].contact_force.wrench.force.z = pressure_value_01_;
      if(pressure_value_01_>(initial_pressure_value+initial_bias[0]))
         contact_msg.foot_contacts[0].is_contact = true;
      else
         contact_msg.foot_contacts[0].is_contact = false;

      contact_msg.foot_contacts[1].contact_force.wrench.force.z = pressure_value_02_;
      if(pressure_value_02_>(initial_pressure_value+initial_bias[1]))
         contact_msg.foot_contacts[1].is_contact = true;
      else
         contact_msg.foot_contacts[1].is_contact = false;

      contact_msg.foot_contacts[2].contact_force.wrench.force.z = pressure_value_03_;
      if(pressure_value_03_>(initial_pressure_value+initial_bias[2]))
         contact_msg.foot_contacts[2].is_contact = true;
      else
         contact_msg.foot_contacts[2].is_contact = false;

      contact_msg.foot_contacts[3].contact_force.wrench.force.z = pressure_value_04_;
      if(pressure_value_04_>(initial_pressure_value+initial_bias[3]))
         contact_msg.foot_contacts[3].is_contact = true;
      else
         contact_msg.foot_contacts[3].is_contact = false;
//      char data_1[4] = {data_frame_copy[6],data_frame_copy[7],data_frame_copy[8],data_frame_copy[9]};
//      char data_2[4] = {data_frame_copy[10],data_frame_copy[11],data_frame_copy[12],data_frame_copy[13]};
//      char data_3[4] = {data_frame_copy[14],data_frame_copy[15],data_frame_copy[16],data_frame_copy[17]};
//      char data_4[4] = {data_frame_copy[18],data_frame_copy[19],data_frame_copy[20],data_frame_copy[21]};
//      char data_5[4] = {data_frame_copy[22],data_frame_copy[23],data_frame_copy[24],data_frame_copy[25]};
//      char data_6[4] = {data_frame_copy[26],data_frame_copy[27],data_frame_copy[28],data_frame_copy[29]};
//      geometry_msgs::WrenchStamped force_in_Newton;
//      float f_x_1 = ByteToFloat(data_1);
//      float f_y_1 = ByteToFloat(data_2);
//      float f_z_1 = ByteToFloat(data_3);

//      force_in_Newton.header.stamp = ros::Time(0);
//      force_in_Newton.wrench.force.x = f_x_1;
//      force_in_Newton.wrench.force.y = f_y_1;
//      force_in_Newton.wrench.force.z = f_z_1;
//      force_pub_1.publish(force_in_Newton);
//      float f_x_2 = ByteToFloat(data_4);
//      float f_y_2 = ByteToFloat(data_5);
//      float f_z_2 = ByteToFloat(data_6);

//      force_in_Newton.header.stamp = ros::Time(0);
//      force_in_Newton.wrench.force.x = f_x_2;
//      force_in_Newton.wrench.force.y = f_y_2;
//      force_in_Newton.wrench.force.z = f_z_2;
//      force_pub_2.publish(force_in_Newton);
      contact_pub_.publish(contact_msg);
      rate.sleep();
    }



}





//std::string PressureSensorRS485Driver::toString(const boost::asio::streambuf& ack_buf)
//{
//  boost::asio::streambuf::const_buffers_type bufs = ack_buf.data();
//  std::string line(boost::asio::buffers_begin(bufs), boost::asio::buffers_end(bufs));
//  return line;
//}

//float PressureSensorRS485Driver::ByteToFloat(char* byteArry)//使用取地址的方法进行处理
//{
//return *((float*)byteArry);
//}

//std::ostream& operator << (std::ostream& out, const boost::asio::streambuf& ack_buf)
//{
////  std::istream is(&ack_buf);
//  boost::asio::streambuf::const_buffers_type bufs = ack_buf.data();
//  std::string line(boost::asio::buffers_begin(bufs), boost::asio::buffers_end(bufs));
//  out<<line;
//  return out;
//}

}
