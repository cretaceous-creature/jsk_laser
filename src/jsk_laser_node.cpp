#include <ros/ros.h>
#include <jsk_laser/serial_comm.h>


int
main(int argc, char** argv)
{
    ros::init(argc, argv, "jsk_laser");
    ros::NodeHandle nh("jsk_laser_serial_comm");
    ros::NodeHandle nhp("~");

    std::string port_str;
    nhp.param("serial_port", port_str, std::string("/dev/ttyUSB0"));

    int baudrate;

    nhp.param("baudrate", baudrate, 2000000);

    SerialComm* comm;
    comm = new SerialComm(nh, nhp);
    if (!comm->open(port_str, baudrate)) { return -1; }

    ros::spin();

    delete comm;

    return 0;
}

