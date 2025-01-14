/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2016, Orbbec Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Tim Liu (liuhua@orbbec.com)
 */

#ifndef ASTRA_DRIVER_H
#define ASTRA_DRIVER_H

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

//#include <sensor_msgs/Image.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <builtin_interfaces/msg/time.hpp>

//#include <dynamic_reconfigure/server.h>
//#include <astra_camera/AstraConfig.h>

//#include <image_transport/image_transport.h>
//#include <camera_info_manager/camera_info_manager.h>

#include <string>
#include <vector>

#include "astra_camera/astra_device_manager.h"
#include "astra_camera/astra_device.h"
#include "astra_camera/astra_video_mode.h"
//#include "astra_camera/GetSerial.h"

//#include <ros/ros.h>
#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include "astra_camera/ros12_shim.h"

namespace astra_wrapper
{

class AstraDriver
{
public:
  //AstraDriver(ros::NodeHandle& n, ros::NodeHandle& pnh) ;
  AstraDriver(rclcpp::Node::SharedPtr& n, rclcpp::Node::SharedPtr& pnh, size_t width, size_t height, double framerate,
              size_t dwidth, size_t dheight, double dframerate, PixelFormat dformat);

private:
  //typedef astra_camera::AstraConfig Config;
  //typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

  void newIRFrameCallback(sensor_msgs::msg::Image::SharedPtr image);
  void newColorFrameCallback(sensor_msgs::msg::Image::SharedPtr image);
  void newDepthFrameCallback(sensor_msgs::msg::Image::SharedPtr image);

  // Methods to get calibration parameters for the various cameras
  sensor_msgs::msg::CameraInfo::SharedPtr getDefaultCameraInfo(int width, int height, double f) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getColorCameraInfo(int width, int height, builtin_interfaces::msg::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getIRCameraInfo(int width, int height, builtin_interfaces::msg::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getDepthCameraInfo(int width, int height, builtin_interfaces::msg::Time time) const;

  void readConfigFromParameterServer();

  // resolves non-URI device IDs to URIs, e.g. '#1' is resolved to the URI of the first device
  std::string resolveDeviceURI(const std::string& device_id); //throw(AstraException);
  void initDevice();

  void advertiseROSTopics();

  void colorConnectCb();
  void depthConnectCb();
  void irConnectCb();

  /// Start the IR stream unless IR streaming is disabled. Because of the
  ///   restriction on IR/RGB simultaneous streaming, IR streaming is initiated
  ///   from multiple methods.
  void irAttemptStream();

  //bool getSerialCb(astra_camera::GetSerialRequest& req, astra_camera::GetSerialResponse& res);

  //void configCb(Config &config, uint32_t level);

  //void applyConfigToOpenNIDevice();

  void genVideoModeTableMap();
  int lookupVideoModeFromDynConfig(int mode_nr, AstraVideoMode& video_mode);

  sensor_msgs::msg::Image::SharedPtr rawToFloatingPointConversion(sensor_msgs::msg::Image::SharedPtr raw_image);

  void setIRVideoMode(const AstraVideoMode& ir_video_mode);
  void setColorVideoMode(const AstraVideoMode& color_video_mode);
  void setDepthVideoMode(const AstraVideoMode& depth_video_mode);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Node::SharedPtr pnh_;

  boost::shared_ptr<AstraDeviceManager> device_manager_;
  boost::shared_ptr<AstraDevice> device_;

  std::string device_id_;

  /** \brief get_serial server*/
  //ros::ServiceServer get_serial_server;

  /** \brief reconfigure server*/
  //boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  bool config_init_;

  std::set<std::string>  alreadyOpen;
  boost::mutex connect_mutex_;
  // published topics
  //image_transport::CameraPublisher pub_color_;
  //image_transport::CameraPublisher pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_camera_info_;
  //image_transport::CameraPublisher pub_depth_raw_;
  //image_transport::CameraPublisher pub_ir_;
  //ros::Publisher pub_projector_info_;

  /** \brief Camera info manager objects. */
  //boost::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_, ir_info_manager_;

  AstraVideoMode ir_video_mode_;
  AstraVideoMode color_video_mode_;
  AstraVideoMode depth_video_mode_;

  std::string ir_frame_id_;
  std::string color_frame_id_;
  std::string depth_frame_id_ ;

  std::string color_info_url_, ir_info_url_;

  bool color_depth_synchronization_;
  bool depth_registration_;

  std::map<int, AstraVideoMode> video_modes_lookup_;

  // dynamic reconfigure config
  double depth_ir_offset_x_;
  double depth_ir_offset_y_;
  int z_offset_mm_;
  double z_scaling_;

  rcl_duration_t ir_time_offset_;
  rcl_duration_t color_time_offset_;
  rcl_duration_t depth_time_offset_;

  int data_skip_;

  int data_skip_ir_counter_;
  int data_skip_color_counter_;
  int data_skip_depth_counter_;

  bool auto_exposure_;
  bool auto_white_balance_;

  bool ir_subscribers_;
  bool color_subscribers_;
  bool depth_subscribers_;
  bool depth_raw_subscribers_;

  /// If false, then camera will never start an IR stream.
  bool can_publish_ir_;
  /// If false, then camera will never start a color stream.
  bool can_publish_color_;
  /// If false, then camera will never start a depth stream.
  bool can_publish_depth_;

  // bool use_device_time_;

  //Config old_config_;
};

}

#endif
