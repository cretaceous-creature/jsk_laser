#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// for ros
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <jsk_laser/JskLaser.h>
#include <jsk_laser/JskLaserRaw.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>

class SerialComm
{
 public:
  SerialComm(ros::NodeHandle nh, ros::NodeHandle nhp_);
  ~SerialComm();

  bool open(const std::string& port_str, int baudrate);

  static const int TIME_SYNC_CALIB_COUNT = 40;
  static const int MAX_PACKET_LEN = 1024;//temp

  static const uint8_t FIRST_HEADER = 0xa2;
  static const uint8_t SECOND_HEADER = 0xa1;
  static const uint8_t FIRST_ENDER = 0x2a;
  static const uint8_t SECOND_ENDER = 0x2b;

  static const uint16_t DATA_SIZE = 548; // 544 + 2(header) + 2(ender)

  static const uint8_t FIRST_HEADER_STAGE = 0x00;
  static const uint8_t SECOND_HEADER_STAGE = 0x01;
  static const uint8_t MSG_DATA_STAGE = 0x02;

  static const float SCALE = 100.0;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher  distances_pub_;
  ros::Publisher  rawdata_pub_;
  ros::Publisher  scan_pub_;

  void readCallback(const boost::system::error_code& error, size_t bytes_transferred);
  void readStart(uint32_t timeout_ms);
  void txCallback(const ros::TimerEvent& timer_event);
  void timeoutCallback(const boost::system::error_code& error);

  boost::asio::io_service comm_uart_service_;
  boost::asio::serial_port comm_port_;
  boost::asio::deadline_timer comm_timer_;
  boost::thread comm_uart_thread_;

  int comm_system_id_;
  int comm_comp_id_;

  unsigned long time_offset;

  uint8_t comm_buffer_[1096];

  uint8_t packet_stage_;
  uint16_t receive_data_size_;
  uint8_t packet_chksum_;
  uint8_t msg_type_;

  ros::Timer tx_timer_;

  bool start_flag_;

  bool comm_timeout_;
  int comm_error_count_;
  bool comm_connected_;

  std::string comm_frame_id_;

  //for ros timestamp sync
  ros::Time time_offest_;
  unsigned long offset_;
  unsigned long offset_tmp_;
  unsigned long sec_offset_;
  unsigned long n_sec_offset_;
  int time_sync_flag_;

  //for the data holder..
#define STEPSIZE 544
#define RAWDATA_NUMBER 5
  std::vector<float> rawdataholder_a[RAWDATA_NUMBER];
  std::vector<float> rawdataholder_b[RAWDATA_NUMBER];  // five 544 data...

  float dist_dataholder[STEPSIZE/2];

  void ProcPubData();

  double poly[7];

  unsigned int pulse_num_buff[RAWDATA_NUMBER];

  bool onlydistdata;
  double lensfocus;
  sensor_msgs::LaserScan laserscan_msg;
  double stable_temperature;
#define SENSOR_LENGTH 5.12
#define PI 3.1415926535898


};

