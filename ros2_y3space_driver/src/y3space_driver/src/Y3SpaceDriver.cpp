#include <Y3SpaceDriver.h>


const std::string Y3SpaceDriver::logger = "[ Y3SpaceDriver ] ";
const std::string Y3SpaceDriver::MODE_ABSOLUTE = "absolute";
const std::string Y3SpaceDriver::MODE_RELATIVE = "relative";

Y3SpaceDriver::Y3SpaceDriver()
    : 
    rclcpp::Node("y3_space_driver")
{
    this->declare_parameter<std::string>("port", "/dev/ttyIMU");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<int>("timeout", 60000);  // 60000
    this->declare_parameter<std::string>("mode", "relative");
    this->declare_parameter<std::string>("frame", "imu_link");

    this->get_parameter("port", port);
    this->get_parameter("baudrate", baudrate);
    this->get_parameter("timeout", timeout);
    this->get_parameter("mode", mode);
    this->get_parameter("frame", frame);

    // printf("IMU 파라미터 확인 === port %s\n", port.c_str());
    // printf("IMU 파라미터 확인 === baudrate %d\n", baudrate);
    // printf("IMU 파라미터 확인 === timeout %d\n", timeout);
    // printf("IMU 파라미터 확인 === mode %s\n", mode.c_str());

    serial_interface_ = std::make_unique<SerialInterface>(port, baudrate, timeout);

    serial_interface_->serialConnect();
    serial_interface_->serialWriteString(SET_AXIS_DIRECTIONS_X_Forward_Y_Right_Z_Up);  // change axis directions
    
    //serial_interface_->serialWriteString(SET_AXIS_DIRECTIONS_X_Right_Y_Up_Z_Forward);
    usleep(1500000);

    this->m_raw_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/raw", 10);
    
    // this->m_raw_imu_pub = this->m_nh.advertise<sensor_msgs::Imu>("/imu/filtered", 10);
    // m_filtered_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", 10);
}

Y3SpaceDriver::~Y3SpaceDriver() {}

void Y3SpaceDriver::restoreFactorySettings()
{
    serial_interface_->serialWriteString(RESTORE_FACTORY_SETTINGS);
}

const std::string Y3SpaceDriver::getSoftwareVersion()
{
    serial_interface_->serialWriteString(GET_FIRMWARE_VERSION_STRING);

    const std::string buf = serial_interface_->serialReadLine();
    RCLCPP_INFO(this->get_logger(), "Software version: %s", buf.c_str());
    return buf;
}

const std::string Y3SpaceDriver::getAxisDirection()
{
    serial_interface_->serialWriteString(GET_AXIS_DIRECTION);

    const std::string buf = serial_interface_->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "X: Right, Y: Up, Z: Forward";
        }
        else if ( buf == "1\r\n")
        {
            return "X: Right, Y: Forward, Z: Up";
        }
        else if ( buf == "2\r\n")
        {
            return "X: Up, Y: Right, Z: Forward";
        }
        else if (buf == "3\r\n")
        {
            return "X: Forward, Y: Right, Z: Up";
        }
        else if( buf == "4\r\n")
        {
            return "X: Up, Y: Forward, Z: Right";
        }
        else if( buf == "5\r\n")
        {
            return "X: Forward, Y: Up, Z: Right";
        }
        else if (buf == "19\r\n")
        {
            return "X: Forward, Y: Left, Z: Up";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Buffer indicates: %s", buf.c_str());
            return "Unknown";
        }
    }();

    RCLCPP_INFO(this->get_logger(), "Axis Direction:  %s", ret.c_str());
    return ret;
}

void Y3SpaceDriver::startGyroCalibration(void)
{
    RCLCPP_INFO(this->get_logger(), "Starting Auto Gyro Calibration...");
    serial_interface_->serialWriteString(BEGIN_GYRO_AUTO_CALIB);

    rclcpp::Duration duration(5.0, 0);
    RCLCPP_INFO(this->get_logger(), "Proceeding");
}

void Y3SpaceDriver::setMIMode(bool on)
{
    if(on)
    {
        serial_interface_->serialWriteString(SET_MI_MODE_ENABLED);
    }
    else
    {
        serial_interface_->serialWriteString(SET_MI_MODE_DISABLED);
    }
}

const std::string Y3SpaceDriver::getCalibMode()
{
    serial_interface_->serialWriteString(GET_CALIB_MODE);

    const std::string buf = serial_interface_->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Bias";
        }
        else if ( buf == "1\r\n")
        {
            return "Scale and Bias";
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Buffer indicates: %s", buf.c_str());
            return "Unknown";
        }
    }();

    RCLCPP_INFO(this->get_logger(), "Calibration Mode:  %s", ret.c_str());
    return ret;
}

const std::string Y3SpaceDriver::getMIMode()
{
    serial_interface_->serialWriteString(GET_MI_MODE_ENABLED);

    const std::string buf = serial_interface_->serialReadLine();
    const std::string ret = [&]()
    {
        if(buf == "0\r\n")
        {
            return "Disabled";
        }
        else if ( buf == "1\r\n")
        {
            return "Enabled";
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Buffer indicates: %s", buf.c_str());
            return "Unknown";
        }
    }();

    RCLCPP_INFO(this->get_logger(), "MI Mode: %s", ret.c_str());
    return ret;
}


//! Run the serial sync
void Y3SpaceDriver::run()
{
    std::vector<double> parsedVals;
    auto imuMsg = sensor_msgs::msg::Imu();
    usleep(300000);

    this->startGyroCalibration();
    this->getSoftwareVersion();
    this->getAxisDirection();
    this->getCalibMode();
    // this->setMIMode(1);
    this->getMIMode();

    if (mode == MODE_ABSOLUTE)
    {
        RCLCPP_INFO(this->get_logger(), "Using absolute driver stream configuration");
        serial_interface_->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_ABSOLUTE);
    }
    else if (mode == MODE_RELATIVE)
    {
        RCLCPP_INFO(this->get_logger(), "Using relative driver stream configuration");
        serial_interface_->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_RELATIVE);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Unknown driver mode set... Defaulting to relative");
        serial_interface_->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_RELATIVE);
    }
    
    
    serial_interface_->serialWriteString(TARE_WITH_CURRENT_ORIENTATION);
    serial_interface_->serialWriteString(TARE_WITH_CURRENT_QUATERNION);
    // usleep(70000);
    serial_interface_->serialWriteString(SET_STREAMING_TIMING_100_MS);
    serial_interface_->serialWriteString(START_STREAMING);

    RCLCPP_INFO(this->get_logger(), "Ready\n");

    rclcpp::Rate rate(10);
    int line = 0;
    while(rclcpp::ok())
    {
        while(serial_interface_->available() > 0)
        {
            line += 1;
            std::string buf = serial_interface_->serialReadLine();
            std::string parse;
            std::stringstream ss(buf);
            double i;

            // Parse data from the line
            while (ss >> i)
            {
                parsedVals.push_back(i);
                if (ss.peek() == ',')
                ss.ignore();
            }

            // Should stop reading when line == number of tracked streams
            if(line == 4)
            {
                // Reset line tracker
                line = 0;
        
                // Prepare IMU message
                imuMsg.header.stamp           =  this->now();
                imuMsg.header.frame_id        = m_frame;
                imuMsg.orientation.x          = parsedVals[0];
                imuMsg.orientation.y          = parsedVals[1];
                imuMsg.orientation.z          = parsedVals[2];
                imuMsg.orientation.w          = parsedVals[3];
                imuMsg.angular_velocity.x     = parsedVals[4];
                imuMsg.angular_velocity.y     = parsedVals[5];
                imuMsg.angular_velocity.z     = parsedVals[6];
                imuMsg.linear_acceleration.x  = parsedVals[7];
                imuMsg.linear_acceleration.y  = parsedVals[8];
                imuMsg.linear_acceleration.z  = parsedVals[9];

                // Clear parsed values
                parsedVals.clear();

                m_raw_imu_pub->publish(imuMsg);
            }
        }

        rate.sleep();
    }
}
