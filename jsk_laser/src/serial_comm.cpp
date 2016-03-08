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

  n_sec_offset_ = 0;
  sec_offset_ = 0;
  offset_ = 0;

  start_flag_ = true;

  packet_stage_ = FIRST_HEADER_STAGE;
  receive_data_size_ = 1;

  time_offset = 0;
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
              receive_data_size_ = DATA_SIZE - 2; // 544 + 2
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
          jsk_laser::JskLaser distances_msg;
          distances_msg.header.stamp = ros::Time::now();

          int first_ender_index = DATA_SIZE - 2 - 2;
          int second_ender_index = DATA_SIZE - 2 - 1;
          if(comm_buffer_[first_ender_index] == FIRST_ENDER &&
             comm_buffer_[second_ender_index] == SECOND_ENDER)
            {
              distances_msg.distances.resize(0);
              for(int i = 0; i < DATA_SIZE - 4; i+=2)
                {
                  short distance;
                  memcpy(&distance, &(comm_buffer_[i]),2);
                  //distances_msg.distances.push_back((float)distance/ SCALE);
                  distances_msg.distances.push_back(distance);
                }
              distances_pub_.publish(distances_msg);
            }
          else
            {
              if(!error_receive_flag)
                {
                  ROS_WARN("wrong ender");
                  error_receive_flag = true;
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
    //ROS_INFO("The bytes is %d", bytes_transferred);

    readStart(1000);
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


