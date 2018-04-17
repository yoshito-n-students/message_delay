#ifndef MESSAGE_DELAY_MESSAGE_DELAY_HPP
#define MESSAGE_DELAY_MESSAGE_DELAY_HPP

#include <nodelet/nodelet.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/timer.h>
#include <topic_tools/shape_shifter.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace message_delay {

class MessageDelay : public nodelet::Nodelet {
public:
  MessageDelay() {}
  virtual ~MessageDelay() {}

private:
  virtual void onInit() {
    // get node handles
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    // load parameters
    const bool unreliable(pnh.param("unreliable", false));
    delay_.fromSec(pnh.param("delay", 1.));

    // subscribe the input topic
    ros::TransportHints transport_hints;
    if (unreliable) {
      // prefer unreliable connection than reliable connection
      transport_hints.unreliable().reliable();
    }
    subscriber_ = nh.subscribe("topic_in", 10, &MessageDelay::onMessageReceived, this, transport_hints);
  }

private:
  void onMessageReceived(const topic_tools::ShapeShifter::ConstPtr &msg) {
    // get node handle
    ros::NodeHandle &nh(getNodeHandle());

    // advertise the output topic if never
    if (!publisher_) {
      publisher_ = msg->advertise(nh, "topic_out", 10);
    }

    // schedule delayed publishment
    ros::Timer *const timer(new ros::Timer);
    *timer = nh.createTimer(delay_, boost::bind(&MessageDelay::onTimerExpired, this, msg, timer),
                            true /*oneshot*/);
  }

  void onTimerExpired(const topic_tools::ShapeShifter::ConstPtr &msg,
                      const ros::Timer *const timer) {
    // publish if the publisher is valid
    if (publisher_) {
      publisher_.publish(msg);
    }

    // remove this expired timer
    delete timer;
  }

private:
  // parameters
  ros::Duration delay_;

  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
};
} // namespace message_delay

#endif // MESSAGE_DELAY_MESSAGE_DELAY_HPP