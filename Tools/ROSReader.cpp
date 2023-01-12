#include "ROSReader.h"


ROSReader::ROSReader(ros::NodeHandle n, std::string color_topic, std::string depth_topic):
LogReader("/temp/elastic_fusion", false), nh(n),
color_sub(nh, color_topic, 10),
depth_sub(nh, depth_topic, 10),
sync(sync_pol(10), color_sub, depth_sub)
{
    depthReadBuffer = new uint8_t[numPixels * 2];
    imageReadBuffer = new uint8_t[numPixels * 3];

    sync.registerCallback(boost::bind(&ROSReader::RGBD_callback, this, _1, _2));

    ROS_INFO("color sub topic: %s, depth sub topic: %s", color_sub.getTopic().c_str(), depth_sub.getTopic().c_str());

    rgb=nullptr;
    depth=nullptr;
}

ROSReader::~ROSReader()
{
    delete[] depthReadBuffer;
    delete[] imageReadBuffer;
}

void ROSReader::getNext() 
{
    // ROS_INFO("get next frame");
    memcpy(imageReadBuffer, color_buffer.front().data.data(), numPixels * 3);
    memcpy(depthReadBuffer, depth_buffer.front().data.data(), numPixels * 2);

    color_buffer.pop_front();
    depth_buffer.pop_front();

    rgb = (uint8_t*)imageReadBuffer;
    depth = (uint16_t*)depthReadBuffer;

    for (int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3) {
      std::swap(rgb[i + 0], rgb[i + 2]);
    }
}

int ROSReader::getNumFrames() 
{
    return numFrames;
}

bool ROSReader::hasMore() 
{
    return (!color_buffer.empty()) && (!depth_buffer.empty());
}

const std::string ROSReader::getFile() 
{
    return file;
}

void ROSReader::setAuto(bool value) 
{
    return;
}

void ROSReader::RGBD_callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth)
{
    // ROS_INFO("got rgbd");
    numFrames++;
    color_buffer.push_back(*color);
    depth_buffer.push_back(*depth);

    lastFrameTime=color->header.stamp.toNSec() / 1.0e3;
    // ROS_INFO("timestamp: %ld us", lastFrameTime);
    timestamp=lastFrameTime;

    while(color_buffer.size() > 10)
    {
        color_buffer.pop_front();
    }
    while(depth_buffer.size() > 10)
    {
        depth_buffer.pop_front();
    }
}