#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
# include <stdio.h>
# include <stdlib.h>
#define pi 3.1415926

using namespace std;
using namespace boost::asio;

int force_count_10 = 0;
unsigned char buf[19]={0};
unsigned char torque_buf[12]={0};//采集的原始数据
float six_axis_force[6] = {0.0,0.0,0.0,0.0,0.0,0.0};//计算的结果
float six_axis_force_init[6] = {0};
float six_axis_force_modify[6] = {0};
float AmpZero[6] = {32713.0,32748.0,32691.0,32692.0,32688.0,32663.0};
float Gain[6] = {123.154096,123.221465,123.282418,123.388284,123.308083,123.295251};
float Ex[6] = {4.969362,4.969362,4.969362,4.969362,4.969362,4.969362};
float decoupled_matrix[6][6]={
	-1.59346,	-0.54408,	6.45183,	269.17674,	1.65611,	-264.83488,
	1.48932,	-309.69772,	-3.67068,	154.07485,	1.48574,	154.81533,
	840.13526,	7.87901,	842.76251,	-3.71399,	838.0854,	-9.79321,
	-0.12696,	-0.0156,	13.96169,	0.00834,	-14.49178,	0.11147,
	-17.00555,	-0.12494,	7.96751,	-0.22989,	7.89847,	0.06416,
	-0.00505,	5.6263,		0.10969,	5.45042,	-0.01368,	5.73402
};

int check_sum(unsigned char *buf){
	int sum = 0;
	for(int i = 6;i < 18;i++){
		sum += buf[i];
		//printf("sum = %d,buf[i] = %d\n",sum,buf[i]);
	}
	//printf("int sum:%d\n",sum);
	sum = (unsigned char) sum;
	//printf("buf[18] = %d,char sum=%d\n",buf[18],sum);
	if(sum == buf[18])
		return 1;
	else
		return 0;
}

int six_axis_calculation(void){
	int returnflag = 0;
	if(check_sum(buf)){
		float six_axis_data[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
		for(int i = 0;i < 6;i++)
			six_axis_force[i] = 0.0;
		for(int i = 0;i < 6;i++){
			six_axis_data[i] = 1000.0*((int)(torque_buf[i*2]<<8|torque_buf[i*2+1]) - AmpZero[i])/65535.0*5/Gain[i]/Ex[i];
		}
		for(int i = 0;i < 6;i++){
			for(int j = 0;j < 6;j++)
				six_axis_force[i] += decoupled_matrix[i][j]*six_axis_data[i]; 
		}
		force_count_10++;
		if(force_count_10 < 10){
			for(int i = 0;i < 6;i++)
				six_axis_force_init[i] = - six_axis_force[i];
		}
		else{
			force_count_10 = 10;
			for(int i = 0;i < 6;i++)
				six_axis_force_modify[i] = six_axis_force[i] + six_axis_force_init[i];
			returnflag = 1;
		}
		}
	else
		returnflag = 0;
	
	return returnflag;
}
float ByteToFloat(char* byteArry)//使用取地址的方法进行处理
{
return *((float*)byteArry);
}

int main(int argc, char* argv[])
{
	std::vector<double> testArray = {1,2,3,4,5,6};
	std_msgs::Float64MultiArray msg;
	int time = 0;

	io_service iosev;
	boost::asio::serial_port *SerialPort;

	//节点文件
	serial_port sp(iosev, "/dev/ttyUSB0");
	// 设置参数
	sp.set_option(serial_port::baud_rate(115200));
	sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
	sp.set_option(serial_port::parity(serial_port::parity::none));
	sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
	sp.set_option(serial_port::character_size(8));

	ros::init(argc, argv, "forcesensor1_pub");
  	ros::NodeHandle n;
	ros::Publisher force_pub = n.advertise<std_msgs::Float64MultiArray>("six_axis_force_1", 1000);
	ros::Rate loop_rate(100);

	char open[8]={0x41,0x54,0x2B,0x47,0x4F,0x44,0x0D,0x0A};
	boost::system::error_code err;
	boost::asio::streambuf ack_buf;
	std::string set_update_rate = "AT+SMPR=100\r\n";
	std::string set_decouple_matrix = "AT+DCPM=(2497.565,0.000000,0.000000,0.000000,0.000000,0.000000);(0.000000,2530.428,0.000000,0.000000,0.000000,0.000000);(0.000000,0.000000,5176.251,0.000000,0.000000,0.000000);(0.000000,0.000000,0.000000,2494.512,0.000000,0.000000);(0.000000,0.000000,0.000000,0.000000,2493.392,0.000000);(0.000000,0.000000,0.000000,0.000000,0.000000,5101.520)\r\n";
	std::string set_compute_unit = "AT+DCPCU=MVPV\r\n";
	std::string set_recieve_format= "AT+SGDM=(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)\r\n";
	std::string get_data_stream = "AT+GSD\r\n";
	std::string stop_data_stream = "AT+GSD=STOP\r\n";
	write(sp, boost::asio::buffer(set_update_rate));
	std::cout<<set_update_rate<<std::endl;
	read_until(sp, ack_buf, "\r\n");
	std::istream is(&ack_buf);
	std::string s;
	is >> s;

	char ack[3];
	std::cout<<s<<std::endl;

	write(sp, boost::asio::buffer(set_decouple_matrix));
	read_until(sp, ack_buf, "\r\n");
	std::cout<<set_decouple_matrix<<std::endl;

	write(sp, boost::asio::buffer(set_compute_unit));
	read_until(sp, ack_buf, "\r\n");

	write(sp, boost::asio::buffer(set_recieve_format));
	read_until(sp, ack_buf, "\r\n");
	std::cout<<set_recieve_format<<std::endl;

	write(sp, boost::asio::buffer(get_data_stream));
	std::cout<<get_data_stream<<std::endl;

	while (ros::ok()){
		// 向串口写数据
//		ROS_INFO("1:%d times",time);
//		write(sp, boost::asio::buffer(get_data_stream));
//		write(sp, buffer(open, 8));
		/*printf("TX:");
		for(int i = 0;i < 8;i++)
			printf(" 0X%02x",open[i]);
		printf("\n");*/

		// 向串口读数据	
//		ROS_INFO("2:%d times",time);
		char data_frame[31];
		std::string data_str;
		size_t len = read(sp, buffer(data_frame,31));
		/* printf("RX:");
		for(int i = 0;i < 19;i++)
			printf(" 0X%02x",buf[i]);
		printf("\n");*/
		
		// for(int i = 0;i < 12;i++)
		// 	torque_buf[i] = buf[i+6];

		// if(six_axis_calculation()){
		// 	/*printf("six axis force are:");
		// 	for(int i = 0;i < 6;i++)
		// 		printf("%10.3f",six_axis_force_modify[i]);
		// 	printf("\n");*/
		// 	ROS_INFO("Force:%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",
		// 			 six_axis_force_modify[0],six_axis_force_modify[1],six_axis_force_modify[2],
		// 			 six_axis_force_modify[3],six_axis_force_modify[4],six_axis_force_modify[5]);
		// 	for(int i = 0;i < 6;i++)
		// 		testArray[i] = six_axis_force_modify[i];	
		// 	msg.data = testArray;
		// 	force_pub.publish(msg);
		// }
		// else
		// 	printf("data error / data initial %d times !!\n",force_count_10);
		ROS_INFO("recieve %d bytes in buffer: \n", int(len));
		for(int i=0;i<len;i++)
		  printf(" %02X",data_frame[i]);
//		  cout<<hex<<int(data_frame[i]);
		cout<<endl;
		char data_1[4] = {data_frame[6],data_frame[7],data_frame[8],data_frame[9]};
		char data_2[4] = {data_frame[10],data_frame[11],data_frame[12],data_frame[13]};
		char data_3[4] = {data_frame[14],data_frame[15],data_frame[16],data_frame[17]};
		float f_x = ByteToFloat(data_1);
		float f_y = ByteToFloat(data_2);
		float f_z = ByteToFloat(data_3);

		std::cout<<"Force in Z is : "<<f_z<<std::endl;
		iosev.run();
		loop_rate.sleep();
		}
	write(sp, boost::asio::buffer(stop_data_stream));

	iosev.stop();
	return 0;
}



// void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
// {
// 	ROS_INFO("I heard vx: [%f]", msg->linear.x);
// 	ROS_INFO("I heard vy: [%f]", msg->linear.y);
// 	ROS_INFO("I heard wz: [%f]", msg->angular.z);
// 	double vx=1000*msg->linear.x;
// 	double vy=1000*msg->linear.y;
// 	double wz=1000*msg->angular.z;
// 	char cmd_vel_serial[10]={0xff,0xfe,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x0f};
//                                               //vx      //vy      //wz      //方向
// 	//vx赋值
// 	int h8x,l8x,vxi;
// 	if(vx>0)cmd_vel_serial[9]&=0x0B; else vx=-vx;
// 	vxi=int(vx);
// 	h8x=vxi/256;
// 	l8x=vxi%256;
// 	//cout<<h8<<endl<<l8<<endl;
// 	cmd_vel_serial[3]+=l8x;
// 	cmd_vel_serial[4]+=h8x;
// 	//if(vx>0)cmd_vel_serial[9]&=0x0B;
// 	//vy赋值
// 	int h8y,l8y,vyi;
// 	if(vy>0)cmd_vel_serial[9]&=0x0D; else vy=-vy;
// 	vyi=int(vy);
// 	h8y=vyi/256;
// 	l8y=vyi%256;
// 	//if(vy>0)cmd_vel_serial[9]&=0x0D;
// 	cmd_vel_serial[5]+=l8y;
// 	cmd_vel_serial[6]+=h8y;
// 	//wz赋值
// 	int h8w,l8w,wzi;
// 	if(wz>0)cmd_vel_serial[9]&=0x0E; else wz=-wz;
// 	wzi=int(wz);
// 	h8w=wzi/256;
// 	l8w=wzi%256;
// 	//if(wz>0)cmd_vel_serial[9]&=0x0E;
// 	cmd_vel_serial[7]+=l8w;
// 	cmd_vel_serial[8]+=h8w;
// 	//cout<<hex<<cmd_vel_serial[9]<<endl;

          
//  //节点文件
// 		/*io_service iosev;
//         serial_port sp1(iosev, "/dev/ttyUSB0");
//         // 设置参数
//         sp1.set_option(serial_port::baud_rate(9600));
//         sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));
//         sp1.set_option(serial_port::parity(serial_port::parity::none));
//         sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
//         sp1.set_option(serial_port::character_size(8));*/
//         size_t data_len=write(sp1, buffer(cmd_vel_serial, 10));
// 		//size_t data_len=write(sp,buffer("hello world", 11));
// 		//write(sp, buffer(cmd_wheel, strlen(cmd_wheel)));
// 		cout<<data_len<<endl;
//      /* char buf[1]={'\0'};
//         read(sp, buffer(buf));
// 	cout<<"result = "<<buf<<endl;*/
// 		iosev.run();
// 		//return 0;
// }

// int main(int argc, char **argv)
// {

//   ros::init(argc, argv, "serial_move");


//   ros::NodeHandle n;
//         sp1.set_option(serial_port::baud_rate(115200));
//         sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));
//         sp1.set_option(serial_port::parity(serial_port::parity::none));
//         sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
//         sp1.set_option(serial_port::character_size(8));
 
//   ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);


//   ros::spin();

//   return 0;
// }

