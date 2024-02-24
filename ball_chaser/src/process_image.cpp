#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    if (!client.call(srv))
    {
        ROS_ERROR("drive_bot service calling failed");
    }
}

void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int index = -1, detect_ball = 0;
    for (int i = 0; i < img.height * img.step; i++)
    {
        if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel)
        {
            index = i % img.step;
            detect_ball = 1;
            break;
        }
        else
        {
            detect_ball = 0;
        }
    }
    if (detect_ball == 1 && index != -1)
    {
        if (index <= img.step * 0.3 && index >= 0)
        {
            drive_robot(0, 0.5);
        }
        else if (index > img.step * 0.7 && index <= img.step)
        {
            drive_robot(0, -0.5);
        }
        else if (index != -1)
        {
            drive_robot(0.5, 0);
        }
    }
    else
    {
        drive_robot(0, 0);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    ros::spin();

    return 0;
}