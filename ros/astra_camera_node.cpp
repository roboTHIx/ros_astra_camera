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

#include "astra_camera/astra_driver.h"

static std::string get_command_option(const std::vector<std::string> &args, const std::string &option)
{
  auto it = std::find(args.begin(), args.end(), option);
  if (it != args.end() && ++it != args.end()) {
    return *it;
  }
  return std::string();
}

static bool get_command_option_exists(const std::vector<std::string> &args, const std::string &option)
{
  return std::find(args.begin(), args.end(), option) != args.end();
}

static bool parse_command_options(int argc, char **argv, size_t *width, size_t *height,
                                  double *framerate, size_t *depth_width, size_t *depth_height,
                                  double *depth_framerate, astra_wrapper::PixelFormat *dformat,
                                  bool *use_ir, bool *use_color, bool *use_depth)
{
  std::vector<std::string> args(argv, argv + argc);

  std::string width_str = get_command_option(args, "-w");
  if (!width_str.empty()) {
    *width = std::stoul(width_str.c_str());
  }

  std::string height_str = get_command_option(args, "-h");
  if (!height_str.empty()) {
    *height = std::stoul(height_str.c_str());
  }

  std::string framerate_str = get_command_option(args, "-f");
  if (!framerate_str.empty()) {
    *framerate = std::stod(framerate_str.c_str());
  }

  std::string dwidth_str = get_command_option(args, "-dw");
  if (!dwidth_str.empty()) {
    *depth_width = std::stoul(dwidth_str.c_str());
  }

  std::string dheight_str = get_command_option(args, "-dh");
  if (!dheight_str.empty()) {
    *depth_height = std::stoul(dheight_str.c_str());
  }

  std::string dframerate_str = get_command_option(args, "-df");
  if (!dframerate_str.empty()) {
    *depth_framerate = std::stod(dframerate_str.c_str());
  }

  std::string dformat_str = get_command_option(args, "-dformat");
  if (!dformat_str.empty()) {
    if (dformat_str == "1MM") {
      *dformat = astra_wrapper::PixelFormat::PIXEL_FORMAT_DEPTH_1_MM;
    }
    else if (dformat_str == "100UM") {
      *dformat = astra_wrapper::PixelFormat::PIXEL_FORMAT_DEPTH_100_UM;
    }
    else {
      std::cerr << "Invalid depth format; must be \"1MM\" or \"100UM\"" << std::endl;
      return false;
    }
  }

  *use_ir = !get_command_option_exists(args, "-I");
  *use_color = !get_command_option_exists(args, "-C");
  *use_depth = !get_command_option_exists(args, "-D");

  return true;
}

int main(int argc, char **argv){

  // RGB
  size_t width = 1280;
  size_t height = 1024;
  double framerate = 30;

  // Depth
  size_t dwidth = 640;
  size_t dheight = 480;
  double dframerate = 30;
  astra_wrapper::PixelFormat dformat = astra_wrapper::PixelFormat::PIXEL_FORMAT_DEPTH_1_MM;

  bool use_ir = true;
  bool use_color = true;
  bool use_depth = true;

  // TODO(clalancette): parsing the command-line options here is temporary until
  // we get parameters working in ROS2.
  if (!parse_command_options(argc, argv, &width, &height, &framerate, &dwidth, &dheight, &dframerate, &dformat, &use_ir, &use_color, &use_depth)) {
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("astra_camera");
  rclcpp::Node::SharedPtr pnh = rclcpp::Node::make_shared("astra_camera_");

  rcl_interfaces::msg::ParameterDescriptor use_ir_description;
  use_ir_description.name = "use_ir";
  use_ir_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  use_ir_description.description = "Should use IR camera";
  use_ir_description.read_only = false;
  use_ir = pnh->declare_parameter("use_ir", use_ir, use_ir_description);

  rcl_interfaces::msg::ParameterDescriptor use_color_description;
  use_color_description.name = "use_color";
  use_color_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  use_color_description.description = "Should use color camera";
  use_color_description.read_only = false;
  use_color = pnh->declare_parameter("use_color", use_color, use_color_description);

  rcl_interfaces::msg::ParameterDescriptor use_depth_description;
  use_depth_description.name = "use_depth";
  use_depth_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  use_depth_description.description = "Should use depth camera";
  use_depth_description.read_only = false;
  use_depth = pnh->declare_parameter("use_depth", use_depth, use_depth_description);

  astra_wrapper::AstraDriver drv(n, pnh, width, height, framerate, dwidth, dheight, dframerate, dformat);

  rclcpp::spin(n);

  return 0;
}
