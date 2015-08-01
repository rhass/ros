#
# Cookbook Name:: ros_test
# Recipe:: catkin_package
#
# Copyright (C) 2015 Ryan Hass
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

catkin_package 'roslint' do
  source_uri 'https://github.com/ros/roslint'
  revision 'master'
end

# This is a sneaky trick from which we can benefit. The rosserial repository
# is not a flat repo, and contains multiple serial packages in addition to a
# "rosserial" metapackage which builds everything. However, there will be times
# when we really only want to build a single component, and all we need to do
# to accomplish this is use the specific component name in the resource name,
# in our case this is "rosserial_arduino."
catkin_package 'rosserial_arduino' do
  source_uri 'https://github.com/ros-drivers/rosserial.git'
  revision 'indigo-devel'
end
