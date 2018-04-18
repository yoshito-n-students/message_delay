#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_image_publisher");
  ros::NodeHandle nh, pnh("~");

  ros::Publisher publisher(nh.advertise< sensor_msgs::Image >("image", 1));

  sensor_msgs::Image image;
  image.height = pnh.param("height", 480);
  image.width = pnh.param("width", 640);
  image.encoding = sensor_msgs::image_encodings::MONO8;
  image.is_bigendian = false;
  image.step = image.width;
  image.data.resize(image.step*image.height, 0);

  ros::Rate rate(pnh.param("fps", 30.));
  while (ros::ok()) {
    image.header.stamp = ros::Time::now();
    publisher.publish(image);
    rate.sleep();
  }

  return 0;
}