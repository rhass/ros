# ros

This cookbook installs ROS.

## Supported Platforms

- Ubuntu 14.04

## Usage

### ros::default

Include `ros` in your cookbooks metadata file, and make use of the ros provider:

```ruby
ros 'indigo' do
  action: [:install, :upgrade]
end
```

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
