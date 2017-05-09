#include <jsk_laser/serial_comm.h>


SerialComm::SerialComm(ros::NodeHandle nh, ros::NodeHandle nhp)
 : comm_port_(comm_uart_service_)
 , nh_(nh), nhp_(nhp)
 , comm_timer_(comm_uart_service_)
 , comm_system_id_(-1)
 , comm_comp_id_(0)
 , comm_timeout_(false)
 , comm_error_count_(0)
 , comm_connected_(false)
{
  distances_pub_ = nh_.advertise<jsk_laser::JskLaser>("/laser_data", 5);
  rawdata_pub_ = nh_.advertise<jsk_laser::JskLaserRaw>("/laserraw_data", 5);
  scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/tiny_laserscan", 5);

  n_sec_offset_ = 0;
  sec_offset_ = 0;
  offset_ = 0;

  start_flag_ = true;

  packet_stage_ = FIRST_HEADER_STAGE;
  receive_data_size_ = 1;

  time_offset = 0;

  std::string laser_name, laser_link;
  nhp.param("laser_name", laser_name , std::string("J"));
  nhp.param("onlydistdata", onlydistdata , false);
  nhp.param("lensfocus", lensfocus , 3.6);
  nhp.param("laser_link", laser_link , std::string("laser_link"));

  laserscan_msg.angle_min = atan(2*lensfocus/SENSOR_LENGTH);
  laserscan_msg.angle_max = PI - laserscan_msg.angle_min;
  laserscan_msg.angle_increment = (PI - 2*laserscan_msg.angle_min)/256;
  laserscan_msg.range_min = 0;
  laserscan_msg.range_max = 3.0;

  laserscan_msg.header.frame_id = laser_link;
  std::cout<<"angle_min: "<<laserscan_msg.angle_min<<std::endl;
  std::cout<<"angle_max: "<<laserscan_msg.angle_max<<std::endl;
  std::cout<<"angle_inc: "<<laserscan_msg.angle_increment<<std::endl;
  std::cout<<"range_min: "<<laserscan_msg.range_min<<std::endl;
  std::cout<<"range_max: "<<laserscan_msg.range_max<<std::endl;


  std::cout<<"laser_name: "<<laser_name<<std::endl;
  std::cout<<"only use distance data?  "<<onlydistdata<<std::endl;
  if(!laser_name.compare(std::string("R")))
  {
      //RED
      poly[0] = 1.648001258107309e+06;
      poly[1] = -5.305835891559632e+06;
      poly[2] = 7.127857366587162e+06;
      poly[3] = -5.110547144753641e+06;
      poly[4] = 2.060999435889212e+06;
      poly[5] = -4.423848603771256e+05 -15;
      poly[6] = 3.937815633389474e+04 + 9;
      stable_temperature=480.0; // 48degree
      temperature_sensi = 0.09623;
  }
  else if(!laser_name.compare(std::string("J")))
  {
      //J
      poly[0] = -7.030060013434869e+07;
      poly[1] = 2.340939930305397e+08;
      poly[2] = -3.244894105718181e+08;
      poly[3] = 2.396546574907680e+08;
      poly[4] = -9.946220641072804e+07;
      poly[5] = 2.199337693620224e+07;
      poly[6] = -2.024360412459942e+06 + 6;
      stable_temperature=460.0; // 48degree
      temperature_sensi = 0.05623;
  }
  else if(!laser_name.compare(std::string("S")))
  {
      //S
      poly[0] = 2.004878860926888e+09;
      poly[1] = -6.155609773756969e+09;
      poly[2] = 7.872203128922240e+09;
      poly[3] = -5.367517768838326e+09;
      poly[4] = 2.057909512255952e+09;
      poly[5] = -4.206589491410656e+08;
      poly[6] = 3.581571264948885e+07 + 6;
      stable_temperature=470.0; // 47degree
      temperature_sensi = 0.09623;
  }
  else if(!laser_name.compare(std::string("K")))
  {
      //K
      poly[0] = 2.147918234375362e+06;
      poly[1] = -7.736068312640203e+06;
      poly[2] = 1.154494186428487e+07;
      poly[3] = -9.137795412258314e+06;
      poly[4] = 4.045684724836031e+06;
      poly[5] = -9.494901699902674e+05;
      poly[6] = 9.219599710241470e+04;
      stable_temperature=440.0; // 48degree
      temperature_sensi = 0.05623;
  }
  else
  {
      //T
      poly[0] = -6.539712337289967e+05;
      poly[1] =  2.522196790614386e+06;
      poly[2] = -3.966911268599921e+06;
      poly[3] = 3.266065347655729e+06;
      poly[4] = -1.487548484816432e+06;
      poly[5] = 3.563561476000117e+05 + 20;
      poly[6] = -3.519452184644648e+04 - 9 + 6;
      stable_temperature=460.0; // 48degree
      temperature_sensi = 0.05623;
  }

}

SerialComm::~SerialComm()
{

}

bool SerialComm::open(const std::string& port_str, int baudrate)
{
    comm_timeout_ = false;
    comm_error_count_ = 0;
    comm_connected_ = false;


    // open port
    try
    {
        comm_port_.open(port_str);

        ROS_INFO("Opened serial port %s.", port_str.c_str());
    }
    catch (boost::system::system_error::exception e)
    {
        ROS_ERROR("Could not open serial port %s. Reason: %s.", port_str.c_str(), e.what());
        return false;
    }

    // configure baud rate
    try
    {
        comm_port_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        comm_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        comm_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        comm_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        comm_port_.set_option(boost::asio::serial_port_base::character_size(8));

        ROS_INFO("Set baudrate %d.", baudrate);
    }
    catch (boost::system::system_error::exception e)
    {
        ROS_ERROR("Could not set baudrate %d. Reason: %s.", baudrate, e.what());

        return false;
    }


    // set up thread to asynchronously read data from serial port
    readStart(1000);
    comm_uart_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &comm_uart_service_));

    comm_connected_ = true;

    /* シグナルハンドラの設定 */
    return true;
}

void SerialComm::readCallback(const boost::system::error_code& error, size_t bytes_transferred)
{
    static bool error_receive_flag = false;
    if (error)
    {
        if (error == boost::asio::error::operation_aborted)
        {
            // if serial connection timed out, try reading again
            if (comm_timeout_)
            {
                comm_timeout_ = false;
                readStart(1000);
                ROS_WARN("restart");
                return;
            }
        }

        ROS_WARN("Read error: %s", error.message().c_str());

        if (comm_error_count_ < 10)
        {
            readStart(1000);
        }
        else
        {
            ROS_ERROR("# read errors exceeded 10. Aborting...");
            ros::shutdown();
        }

        ++comm_error_count_;

        return;
    }

    comm_timer_.cancel();

    switch(packet_stage_)
    {

    case FIRST_HEADER_STAGE:
    {
        if(comm_buffer_[0] == FIRST_HEADER)
        {
            packet_stage_ = SECOND_HEADER_STAGE;
            receive_data_size_ = 1;
            //ROS_INFO("first header ok");
        }
        else
        {
            packet_stage_ = FIRST_HEADER_STAGE;
            receive_data_size_ = 1;
            if(!error_receive_flag)
            {
                ROS_ERROR("first header bad");
                error_receive_flag = true;
            }
        }
        break;
    }
    case SECOND_HEADER_STAGE:
    {
        if(comm_buffer_[0] == SECOND_HEADER)
        {
            packet_stage_ = MSG_DATA_STAGE;
            if(onlydistdata) // only distance data are acquired..
                receive_data_size_ = DATA_SIZE  - 2; // 548 - 2 = 546
            else
                receive_data_size_ = DATA_SIZE * 2 - 2; // 1096 -2 = 1094
        }
        else
        {
            packet_stage_ = FIRST_HEADER_STAGE;
            receive_data_size_ = 1;
            if(!error_receive_flag)
            {
                ROS_ERROR("second header bad");
                error_receive_flag = true;
            }
        }
        break;
    }
    case MSG_DATA_STAGE:
    {
        //only distance
        if(onlydistdata) // only distance data are acquired..
        {
            jsk_laser::JskLaser laserdata_msg;
            laserdata_msg.header.stamp = ros::Time::now();
            laserscan_msg.header.stamp = ros::Time::now();
            int first_ender_index = DATA_SIZE - 2 - 2;
            int second_ender_index = DATA_SIZE - 2 - 1;
            if(comm_buffer_[first_ender_index] == FIRST_ENDER &&
                    comm_buffer_[second_ender_index] == SECOND_ENDER)
            {
                laserdata_msg.distances.resize(0);
                laserdata_msg.reflectance.resize(0);
                for(int i = 0; i < DATA_SIZE - 4; i+=2)
                {
                    short distance;
                    memcpy(&distance, &(comm_buffer_[i]),2);
                    //distances_msg.distances.push_back((float)distance/ SCALE);
                    laserdata_msg.distances.push_back(distance);   //only distance
                }
                distances_pub_.publish(laserdata_msg);
            }
        }
        else//5 pulses data..  need to calculate distance here..
        {
            //minus the first two bytes we have  2(num) + 2(pulse) + 544 * 2 + 2(ender) = 1094
            int first_ender_index = DATA_SIZE * 2 - 2 - 2;  // should be 1092 = 1096 - 4
            int second_ender_index = DATA_SIZE * 2 - 2 - 1; // should be 1093 = 1096 - 3
            if(comm_buffer_[first_ender_index] == FIRST_ENDER &&
                    comm_buffer_[second_ender_index] == SECOND_ENDER)
            {
                uint16_t num, pulse_num;
                memcpy(&num, &(comm_buffer_[0]),2);
                memcpy(&pulse_num, &(comm_buffer_[2]),2);
                pulse_num_buff[num%100] = pulse_num;
                for(int i = 4; i < DATA_SIZE; i+=2)
                {
                    //  A: data[4] ~ data [544+4]
                    //   B: data[544 + 4] ~ data[544 + 4 + 544]
                    short rawdata;
                    if(num > 100)
                    {
                        num -= 100;
                        memcpy(&rawdata, &(comm_buffer_[i]),2);
                        rawdataholder_a[num].push_back((float)rawdata / 10);
                        memcpy(&rawdata, &(comm_buffer_[i + 544]),2);
                        rawdataholder_b[num].push_back((float)rawdata / 10);
                        //all data received,  now we can process and publish... lol
                        ProcPubData();
                    }
                    else
                    {
                        memcpy(&rawdata, &(comm_buffer_[i]),2);
                        rawdataholder_a[num].push_back((float)rawdata / 10);
                        memcpy(&rawdata, &(comm_buffer_[i + 544]),2);
                        rawdataholder_b[num].push_back((float)rawdata / 10);
                    }
                }
            }
            else
            {
                if(!error_receive_flag)
                {
                    ROS_WARN("wrong ender");
                    error_receive_flag = true;
                }
            }
        }

        packet_stage_ = FIRST_HEADER_STAGE;
        receive_data_size_ = 1;
        break;
    }
    default:
    {
        packet_stage_ = FIRST_HEADER_STAGE;
        receive_data_size_ = 1;
        break;
    }
    }


    readStart(1000);
}
/**************************
 * Process and Publish the data...
 *
 * ************************/

void SerialComm::ProcPubData()
{
    jsk_laser::JskLaser laserdata_msg;
    jsk_laser::JskLaserRaw rawdata_msg;
    laserscan_msg.header.stamp = ros::Time::now();
    laserdata_msg.header.stamp = ros::Time::now();
    rawdata_msg.header.stamp = ros::Time::now();
    rawdata_msg.stepsize = STEPSIZE;
    for(int j = 0; j < RAWDATA_NUMBER; j++)
    {
        if(rawdataholder_a[j].size())
            rawdata_msg.num_array.push_back(j);
        for(int i = 0; i < rawdataholder_a[j].size(); i++)
        {
            rawdata_msg.data_a.push_back(rawdataholder_a[j].at(i));
            rawdata_msg.data_b.push_back(rawdataholder_b[j].at(i));

            double bot = rawdataholder_a[j].at(i);
            double top = rawdataholder_b[j].at(i);

            double k = top/(bot+top);
            double dist;
            if(bot>240||bot<0)
                dist= 0;
            else if(bot+top>10 + (pulse_num_buff[j]%100)*0.4){  //here should be + TIMERCOUNTER * 0.4
                float temperature =  (stable_temperature - (int)(pulse_num_buff[0]/100)) * temperature_sensi;  //last part is the temperature compensate
                //temperature = temperature <-2?-2:temperature;
                dist = poly[0]*k*k*k*k*k*k + poly[1]*k*k*k*k*k + poly[2]*k*k*k*k
                        + poly[3]*k*k*k + poly[4]*k*k + poly[5]*k + poly[6] + temperature;
            }
            else
                dist = 0;
            //dist cant be minus
            dist = dist>0?dist:0;

            if(bot>10&&top>10
               &&bot<230&&top<230&&dist!=0){
                //distance unity is cm
                dist_dataholder[i] = dist;  // better to calculate all the data and do average
                reflectance_dataholder[i] = dist * dist * (bot + top) / (pulse_num_buff[j] % 100);
            }
        }
        rawdataholder_a[j].clear();
        rawdataholder_b[j].clear();
        dist_dataholder[175] = dist_dataholder[176];

        if(j==RAWDATA_NUMBER-1)
        {
            laserscan_msg.ranges.clear();
            laserscan_msg.intensities.clear();
            for(int k = 0; k<STEPSIZE/2;k++)
            {
                laserdata_msg.distances.push_back(dist_dataholder[k]);
                laserdata_msg.reflectance.push_back(reflectance_dataholder[k]);
                if(k>=8&&k<(STEPSIZE/2-8)){
                  laserscan_msg.ranges.push_back(dist_dataholder[k]/100);
                  laserscan_msg.intensities.push_back(reflectance_dataholder[k]/5000>1?1:reflectance_dataholder[k]/5000);
                }
                dist_dataholder[k] = 0; //clear this buffer
                reflectance_dataholder[k] = 0; //clear this buffer
            }
            rawdata_pub_.publish(rawdata_msg);
            distances_pub_.publish(laserdata_msg);
            scan_pub_.publish(laserscan_msg);

        }
    }
}


void SerialComm::readStart(uint32_t timeout_ms)
{
  boost::asio::async_read(comm_port_,boost::asio::buffer(comm_buffer_, receive_data_size_),
                          boost::bind(&SerialComm::readCallback, this, 
                                      boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));

  if (timeout_ms != 0)
    {
      comm_timer_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
      comm_timer_.async_wait(boost::bind(&SerialComm::timeoutCallback, this, boost::asio::placeholders::error));
    }
}


void SerialComm::timeoutCallback(const boost::system::error_code& error)
{
    if (!error)
    {
        comm_port_.cancel();
        comm_timeout_ = true;
        ROS_WARN("Serial connection timed out.");
    }
    //ROS_WARN("Read error: %s", error.message().c_str());
}



