#ifndef MESSAGE_DELAY_MESSAGE_DELAY_HPP
#define MESSAGE_DELAY_MESSAGE_DELAY_HPP

#include <nodelet/nodelet.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/timer.h>
#include <topic_tools/shape_shifter.h>

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

namespace message_delay {

class MessageDelay : public nodelet::Nodelet {
public:
  MessageDelay() : guard_timer_(timer_service_) {
    // run timer service in background.
    // the guard timer keeps the service running.
    // the guard timer is preferred than io_service::work
    // because io_service::work is deprecated in recent versions of boost.
    guard_timer_.async_wait(boost::bind(doNothing));
    timer_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &timer_service_));
  }
  virtual ~MessageDelay() {
    // stop the timer serivce
    timer_service_.stop();
    if (timer_thread_.joinable()) {
      timer_thread_.join();
    }
  }

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
    subscriber_ =
        nh.subscribe("topic_in", 10, &MessageDelay::onMessageReceived, this, transport_hints);
  }

private:
  void onMessageReceived(const topic_tools::ShapeShifter::ConstPtr &msg) {
    // get node handle
    ros::NodeHandle &nh(getNodeHandle());

    // advertise the output topic if never
    if (!publisher_) {
      publisher_ = msg->advertise(nh, "topic_out", 10);
    }

    // schedule delayed publishment.
    // ros::Timer can do the same thing
    // but boost::asio::deadline_timer is 50% lighter on my test environment.
    boost::asio::deadline_timer *const timer(new boost::asio::deadline_timer(timer_service_));
    timer->expires_from_now(delay_.toBoost());
    timer->async_wait(boost::bind(&MessageDelay::onTimerExpired, this, msg, timer));
  }

  void onTimerExpired(const topic_tools::ShapeShifter::ConstPtr &msg,
                      const boost::asio::deadline_timer *const timer) {
    // publish if the publisher is valid
    if (publisher_) {
      publisher_.publish(msg);
    }

    // remove this expired timer
    delete timer;
  }

  // for the guard timer
  static void doNothing() {}

private:
  // parameters
  ros::Duration delay_;

  // ros workers
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;

  // timer contexts
  boost::asio::io_service timer_service_;
  boost::asio::deadline_timer guard_timer_;
  boost::thread timer_thread_;
};
} // namespace message_delay

#endif // MESSAGE_DELAY_MESSAGE_DELAY_HPP