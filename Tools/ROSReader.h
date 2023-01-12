#ifndef ROSREADER_H_
#define ROSREADER_H_

#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <deque>

#include "../Core/Utils/Parse.h"

#include "CameraInterface.h"
#include "LogReader.h"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

class ROSReader : public LogReader {
 public:

  ROSReader(ros::NodeHandle n, std::string color_topic, std::string depth_topic);

  virtual ~ROSReader();

  void getNext();

  int getNumFrames();

  bool hasMore();

  bool rewound() {
    return false;
  }

  void rewind() {}

  void getBack() {}

  void fastForward(int frame) {}

  const std::string getFile();

  void setAuto(bool value);

 private:
  void RGBD_callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth);
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> color_sub;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync;

  //unit: us 1e-6s
  int64_t lastFrameTime;
  int lastGot;

  std::deque<sensor_msgs::Image> color_buffer;
  std::deque<sensor_msgs::Image> depth_buffer;
};




#endif //ROSREADER_H_