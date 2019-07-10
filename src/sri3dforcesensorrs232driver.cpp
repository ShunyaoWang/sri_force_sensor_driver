#include "force_sensor/sri3dforcesensorrs232driver.h"
namespace sri_driver {

SRI3DForceSensorRS232Driver::SRI3DForceSensorRS232Driver(const ros::NodeHandle& node_handle,
                                                         const string serial_name, const unsigned int buad_rate)
  : node_handle_(node_handle),
    serial_name_(serial_name),
    buad_rate_(buad_rate),
    update_rate(100),
    chanels_(1)
{
  ROS_INFO("Driver Construct");
  sp = new serial_port(io_sev_);
  if(sp)
    {
      SerialInitialize();
    }
  if(!node_handle_.getParam("/channels", chanels_))
    ROS_WARN("can't load parameters of channel number, default is 1 channel.");

  force_pub_1 = node_handle_.advertise<geometry_msgs::WrenchStamped>("force_and_torque_ch1", 1);
  data_size_ = 31;
  if(chanels_ == 2)
    {
      force_pub_2 = node_handle_.advertise<geometry_msgs::WrenchStamped>("force_and_torque_ch2", 1);

    }
  data_frame_.resize(data_size_);
}

SRI3DForceSensorRS232Driver::~SRI3DForceSensorRS232Driver()
{
  write(*sp, boost::asio::buffer(stop_data_stream));
  std::cout<<stop_data_stream<<std::endl;
  if(sp){
      delete sp;
    }
}

void SRI3DForceSensorRS232Driver::start()
{

  SensorInitialize();
  ROS_INFO("Start Sensor");
  write(*sp, boost::asio::buffer(get_data_stream));
  std::cout<<get_data_stream<<std::endl;
  sensor_read_thread_ = boost::thread(boost::bind(&SRI3DForceSensorRS232Driver::SensorReadThread, this));
  data_process_thread_ = boost::thread(boost::bind(&SRI3DForceSensorRS232Driver::DataProscessAndPublishThread, this));

}

bool SRI3DForceSensorRS232Driver::SerialInitialize()
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

void SRI3DForceSensorRS232Driver::SensorInitialize()
{
  ROS_INFO("Initailizing Sensor");
  write(*sp, boost::asio::buffer(set_zero));
  std::cout<<set_zero<<std::endl;
  read_until(*sp, ack_buf_, "\r\n");
  std::cout << toString(ack_buf_) <<std::endl;

  write(*sp, boost::asio::buffer(set_update_rate));
  std::cout<<set_update_rate<<std::endl;
  read_until(*sp, ack_buf_, "\r\n");
  std::cout << toString(ack_buf_) <<std::endl;

  write(*sp, boost::asio::buffer(set_decouple_matrix));
  read_until(*sp, ack_buf_, "\r\n");
  std::cout<<set_decouple_matrix<<std::endl;
  std::cout << toString(ack_buf_) <<std::endl;

  write(*sp, boost::asio::buffer(set_compute_unit));
  read_until(*sp, ack_buf_, "\r\n");
  std::cout<<set_compute_unit<<std::endl;
  std::cout << toString(ack_buf_) <<std::endl;

  write(*sp, boost::asio::buffer(set_recieve_format));
  read_until(*sp, ack_buf_, "\r\n");
  std::cout<<set_recieve_format<<std::endl;
  std::cout << toString(ack_buf_) <<std::endl;


}

void SRI3DForceSensorRS232Driver::SensorReadThread()
{
  ROS_INFO("Get in Sensor Read Thread");
  ros::Rate rate(update_rate);
  while (ros::ok()) {
      char data_frame[data_size_];
      size_t len = read(*sp, buffer(data_frame,data_size_));
      ROS_INFO("recieve %d bytes in buffer: \n", int(len));
      boost::recursive_mutex::scoped_lock lock(r_mutex_);
      for(int i=0;i<len;i++){
        data_frame_[i] = data_frame[i];
//        printf(" %02X",data_frame_[i]);
        }
      lock.unlock();
//      boost::recursive_mutex::scoped_lock unlock(r_mutex_);
      rate.sleep();
    }
  write(*sp, boost::asio::buffer(stop_data_stream));
  std::cout<<stop_data_stream<<std::endl;
}

bool SRI3DForceSensorRS232Driver::DataProscessAndPublishThread()
{
  ROS_INFO("Get in Data Process and Publish Thread");
  ros::Rate rate(100);
  while (ros::ok()) {
      boost::recursive_mutex::scoped_lock lock(r_mutex_);
      std::vector<char> data_frame_copy = data_frame_;
//      for(int i=0;i<data_size_;i++)
//        printf(" %02X",data_frame_copy[i]);
      lock.unlock();
      geometry_msgs::WrenchStamped force_in_Newton;
      char data_1[4] = {data_frame_copy[6],data_frame_copy[7],data_frame_copy[8],data_frame_copy[9]};
      char data_2[4] = {data_frame_copy[10],data_frame_copy[11],data_frame_copy[12],data_frame_copy[13]};
      char data_3[4] = {data_frame_copy[14],data_frame_copy[15],data_frame_copy[16],data_frame_copy[17]};
      float f_x_1 = ByteToFloat(data_1);
      float f_y_1 = ByteToFloat(data_2);
      float f_z_1 = ByteToFloat(data_3);

      force_in_Newton.header.stamp = ros::Time(0);
      force_in_Newton.wrench.force.x = f_x_1;
      force_in_Newton.wrench.force.y = f_y_1;
      force_in_Newton.wrench.force.z = f_z_1;
      force_pub_1.publish(force_in_Newton);
      if(chanels_ == 2)
        {
          char data_4[4] = {data_frame_copy[18],data_frame_copy[19],data_frame_copy[20],data_frame_copy[21]};
          char data_5[4] = {data_frame_copy[22],data_frame_copy[23],data_frame_copy[24],data_frame_copy[25]};
          char data_6[4] = {data_frame_copy[26],data_frame_copy[27],data_frame_copy[28],data_frame_copy[29]};
          float f_x_2 = ByteToFloat(data_4);
          float f_y_2 = ByteToFloat(data_5);
          float f_z_2 = ByteToFloat(data_6);

          force_in_Newton.header.stamp = ros::Time(0);
          force_in_Newton.wrench.force.x = f_x_2;
          force_in_Newton.wrench.force.y = f_y_2;
          force_in_Newton.wrench.force.z = f_z_2;
          force_pub_2.publish(force_in_Newton);

        }


      rate.sleep();
    }



}

std::string SRI3DForceSensorRS232Driver::toString(const boost::asio::streambuf& ack_buf)
{
  boost::asio::streambuf::const_buffers_type bufs = ack_buf.data();
  std::string line(boost::asio::buffers_begin(bufs), boost::asio::buffers_end(bufs));
  return line;
}

float SRI3DForceSensorRS232Driver::ByteToFloat(char* byteArry)//使用取地址的方法进行处理
{
return *((float*)byteArry);
}

std::ostream& operator << (std::ostream& out, const boost::asio::streambuf& ack_buf)
{
//  std::istream is(&ack_buf);
  boost::asio::streambuf::const_buffers_type bufs = ack_buf.data();
  std::string line(boost::asio::buffers_begin(bufs), boost::asio::buffers_end(bufs));
  out<<line;
  return out;
}

}
