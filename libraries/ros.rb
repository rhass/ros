#
# Cookbook Name:: ros
# Library:: ros
#
# Copyright 2013, Noah Kantrowitz
# Copyright (C) 2015, Ryan Hass
#
# Portions of this code were sourced from the poise-ruby project, written
# by Noah Kantrowitz. 
# https://github.com/poise/poise-ruby
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

require 'poise'
require 'chef/resource'
require 'chef/provider'

module Ros
  class Resource < Chef::Resource
    include Poise
    provides(:ros)
    actions(:install, :upgrade, :remove)

    attribute(:release, kind_of: String, name_attribute: true)
    attribute(:config, kind_of: String, :equal_to => ['desktop-full', 'desktop', 'ros-base', 'ros-core'], default: 'ros-base')
    attribute(:apt_uri, kind_of: String, default: 'http://packages.ros.org/ros/ubuntu')
    attribute(:apt_components, kind_of: Array, default: ['main'])
    attribute(:apt_key, kind_of: String, default: 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key')
    attribute(:sys_profile, kind_of: [TrueClass, FalseClass], default: true)
  end

  class Provider < Chef::Provider
    include Poise
    provides(:ros)

    def action_install
      converge_by("installing #{ros_release}") do
        notifying_block do
          install_repository
          install_package
          initialize_rosdep
        end
      end
    end

    def action_upgrade
      converge_by("upgrading #{ros_release}") do
        notifying_block do
          install_repository
          upgrade_package
        end
      end
    end

    def action_remove
      converge_by("removing #{ros_release}") do
        notifying_block do
          install_repository
          remove_package
        end
      end
    end

    private

    def ros_release
      @ros_release = "ros-#{new_resource.release}-#{new_resource.config}"
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
      elsif node['platform'] == 'debian' && node['platform_release'].start_with?('6.')
        # Debian 6 doesn't install /etc/lsb-release by default so ohai has no data for it
        'squeeze'
      end
      apt_repository 'ros'do
        uri new_resource.apt_uri
        distribution codename
        components new_resource.apt_components
        key new_resource.apt_key
      end

      # Ensure local repo cache is up to date for ROS dependencies.
      include_recipe 'apt::default'
    end

    def initialize_rosdep
      include_recipe 'locale::default'

      execute 'rosdep-init' do
        command 'sudo rosdep init'
        not_if { ::File.exists?('/etc/ros/rosdep/sources.list.d/20-default.list') }
      end
    end

    def sys_profile_manage(cmd)
      profile = "/etc/profile.d/#{new_resource.release}.sh"
      node.default['ros'][new_resource.release]['sys_profile'] = profile

      link profile do
        to "/opt/ros/#{new_resource.release}/setup.sh"
        action cmd
      end
    end

    def install_package
      package ros_release do # ~FC005
        action :install
      end

      if new_resource.sys_profile == true
        sys_profile_manage(:create)
      else
        sys_profile_manage(:delete)
      end
    end

    def upgrade_package
      package ros_release do
        action :upgrade
      end
    end

    def remove_package
      package ros_release do
        action :remove
      end

      sys_profile_manage(:delete)
    end

  end
end
