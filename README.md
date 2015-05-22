# ros

[![Build Status](https://travis-ci.org/rhass/ros.svg)](https://travis-ci.org/rhass/ros)
[![Cookbook Version](https://img.shields.io/cookbook/v/ros.svg)](https://supermarket.chef.io/cookbooks/ros)
[![License](https://img.shields.io/badge/license-Apache_2-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0)

This cookbook installs ROS.

## Supported Platforms

- Ubuntu 14.04

## Usage

Include `ros` in your cookbooks metadata file, and make use of the ros provider:

### Installing ROS
```ruby
ros 'indigo' do
  config 'ros-desktop'
  action [:install, :upgrade]
end
```

### Installing a Catkin Package
``` ruby
catkin_package 'roslint' do
  source_uri 'https://github.com/ros/roslint'
end
```

## Sponsors
Hosting sponsorship is provided by DigitalOcean and your support.
By signing up with the following reference code you are contributing
$25 in hosting time to this project, while getting yourself $10 in
credit. We appreciate your support!

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
