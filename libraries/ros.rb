#
# Cookbook Name:: ros
# Library:: ros_install
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
class Chef

  class Resource::Ros < Resource
    include Poise
    actions(:install, :upgrade, :remove)

    attribute(:version, kind_of: String, name_attribute: true)
    attribute(:config, kind_of: String, :equal_to => ['desktop-full', 'desktop', 'ros-base'], default: 'ros-base')
    attribute(:apt_uri, kind_of: String, default: 'http://packages.ros.org/ros/ubuntu')
    attribute(:apt_components, kind_of: Array, default: ['main'])
    attribute(:apt_key, kind_of: String, default: 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key')
  end

  class Provider::Ros < Provider
    include Poise
    # Work-around for poise issue #8
    include Chef::DSL::Recipe

    def action_install
      converge_by("installing #{ros_package}") do
        notifying_block do
          install_repository
          install_package
        end
      end
    end

    def action_upgrade
      converge_by("upgrading #{ros_package}") do
        notifying_block do
          install_repository
          upgrade_package
        end
      end
    end

    def action_remove
      converge_by("removing #{ros_package}") do
        notifying_block do
          install_repository
          remove_package
        end
      end
    end

    private

    def ros_package
      @ros_package = "ros-#{new_resource.version}-#{new_resource.config}"
    end

    def install_repository
      if node.platform_family?('rhel')
        install_yum_repository
      elsif node.platform_family?('debian')
        install_apt_repository
      else
        raise "Unsupported platform #{node['platform']}"
      end
    end

    def install_yum_repository
      raise NotImplementedError, 'Not there yet'
    end

    def install_apt_repository
      codename = if node['lsb']['codename']
        node['lsb']['codename']
      elsif node['platform'] == 'debian' && node['platform_version'].start_with?('6.')
        # Debian 6 doesn't install /etc/lsb-release by default so ohai has no data for it
        'squeeze'
      end
      apt_repository 'ros'do
        uri new_resource.apt_uri
        distribution codename
        components new_resource.apt_components
        key new_resource.apt_key
      end
    end

    def install_package
      package ros_package do
        action :install
      end
    end

    def upgrade_package
      package ros_package do
        action :upgrade
      end
    end

    def remove_package
      package ros_package do
        action :remove
      end
    end

  end
end
