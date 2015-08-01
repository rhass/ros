# ros

[![Build Status](https://travis-ci.org/rhass/ros.svg)](https://travis-ci.org/rhass/ros)
[![Cookbook Version](https://img.shields.io/cookbook/v/ros.svg)](https://supermarket.chef.io/cookbooks/ros)
[![License](https://img.shields.io/badge/license-Apache_2-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

This cookbook installs ROS.

## Supported Platforms

- Ubuntu 14.04

## Usage

Include `ros` in your cookbooks metadata file, and make use of the ros provider:

## Resources

### ros
The `ros` resource installs a named  version of ROS.

#### Attributes

- `release`         - Name of release to install. _Default: Name of Resource Instance_
- `config`          - Supported values are `desktop-full`, `desktop`, `ros-base`, and `ros-core`. _Default: `ros-base`_
- `apt_uri`         - URI to apt repository for ROS Debian/Ubuntu packages. _Default: `'http://packages.ros.org/ros/ubuntu'`_
- `apt_components`  - Array of apt repo components. _Default: `['main']`_
- `apt_key`         - URI to repository signing key. _Default: `'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key'`_
- `sys_profile`     - Boolean to create symlink of setup.sh into /etc/profile.d - _Default: `true`_

#### Actions
- `:install` - _Default Action_
- `:remove`

#### Usage

```ruby
ros 'indigo' do
  config 'ros-desktop'
  action [:install, :upgrade]
end
```

### catkin
The `catkin` resource is responsible for creating a catkin workspace. This resource must currently be called prior to using catkin_package.

#### Attributes

- `user`              - Owner of catkin workspace. _Default: `node['current_user']`_
- `release`           - Required option to define which version of ROS to bind the workspace.
- `workspace`         - Path of workspace you wish to create. _Default: Name of Resource Instance_
- `workspace_src_dir  - Path of src directory in the workspace. _Default: `"#{workspace}/src"`_
- `ros_path`          - This is the path to where ros is installed. _Default: `/opt/ros/#{release}`_
- `ros_cmd`           - Path to env.sh script. This is used to ensure commands are envoked with the correct environment settings. The default is typically the `install` directory in the workspace, however if the install directory is not present it will use the env.sh in the ros_path.

#### Actions
- `:create` -  _Default Action_
- `:remove`

##### Usage
``` ruby
catkin_package 'roslint' do
  source_uri 'https://github.com/ros/roslint'
end
```

### catkin_package
The `catkin_package` resouce allows you to idempotently build and install catkin packages from git sources.

#### Attributes

- `source_uri`        - URI to Git repository. _Default: Name of Resource Instance_
- `revision`          - Git tag, branch, or revision to checkout. _Default: "#{release}-devel"_

#### Actions
- `:install` - _Default Action_

##### Usage

Installing a package from a flat repository:
```ruby
catkin_package 'roslint' do
  source_uri 'https://github.com/ros/roslint'
  revision 'master'
end
```

Installing a package from a nested directory sturcture:
```ruby
catkin_package 'rosserial_arduino' do
  source_uri 'https://github.com/ros-drivers/rosserial.git'
end
```

### ros_service
The `ros_service` resource utilizes supervisord to manage roscore and nodes as
system services, and creates the appropriate supervisord configuration.

#### Attributes
- `service_name`        - Name of node, driver, or service you wish to have autostart. _Default: Name of Resource Instance_
- `options`             - String of arguments passed to node or driver. _Default: `NilClass`_

#### Actions
- `enable` - _Default Action_
- `disable`
- `start`
- `stop`
- `restart`
- `reload`

##### Usage

Managing roscore:
```ruby
ros_service 'roscore'
```

Starting a driver:
```ruby
ros_service 'xv_11_laser_driver' do
  options 'neato_laser_publisher _port:=/dev/ttyUSB0'
end
```

## Sponsors
Hosting sponsorship is provided by supporters like you. We use DigitalOcean
and your support by signing up with the following reference code you will get
$10 in free credit, and the first $25 of your usages are graciously contributed
to compute time for this project. We appreciate your support!

https://www.digitalocean.com/?refcode=775b6507a7eb

## License and Authors

Author:: Ryan Hass (<ryan@invalidchecksum.net>)

Copyright (c) 2015, Ryan Hass

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
