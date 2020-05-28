#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "gl_driver.h"


int main(int argc, char** argv)
{
    std::string serial_port_name = std::string("/dev/ttyUSB0");
    std::string frame_id = std::string("base");
    std::string pub_topicname_lidar = std::string("scan");
    
    ros::init(argc, argv, "gl_ros_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param("serial_port_name", serial_port_name, serial_port_name);
    nh_priv.param("frame_id", frame_id, frame_id);
    nh_priv.param("pub_topicname_lidar", pub_topicname_lidar, pub_topicname_lidar);

    ros::Publisher data_pub = nh.advertise<sensor_msgs::LaserScan>(pub_topicname_lidar, 100);

    Gl gl(serial_port_name,921600);
    gl.SetFrameDataEnable(1);

    std::cout << "Serial Num : " << gl.GetSerialNum() << std::endl;

    ros::Rate loop_rate(40);
    while(ros::ok())
    {
        sensor_msgs::LaserScan scan_msg;

        Gl::framedata_t frame_data = gl.ReadFrameData();
        int num_data = frame_data.distance.size();
        if(num_data>0)
        {
            scan_msg.header.stamp = ros::Time::now();
            scan_msg.header.frame_id = frame_id;
            scan_msg.angle_min = frame_data.angle[0];
            scan_msg.angle_max = frame_data.angle[num_data-1];
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(num_data-1);
            scan_msg.range_min = 0.1;
            scan_msg.range_max = 30.0;
            scan_msg.ranges.resize(num_data);
            for(int i=0; i<num_data; i++)
            {
                scan_msg.ranges[i] = (double)frame_data.distance[i]/1000.0;
            }
            
            data_pub.publish(scan_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    gl.SetFrameDataEnable(0);

    return 1;
}
