# message_delay
A ROS nodelet to simulate network delay over a topic

## Subscribed Topics
topic_in (any message type)

## Published Topics
topic_out (same message type to topic_in)

## Parameters
~delay (double, defalut: 1.0)
* delay between subscribed and published message in seconds

~unreliable (bool, default: false)
* use unreliable connection in subscribing topic_in

## Examples
see [launch/test.launch](launch/test.launch)
