#
# Cookbook Name:: ros
# Library:: ros_service
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

require 'poise'
require 'chef/resource'
require 'chef/provider'

module RosService
  class Resource < Chef::Resource
    include Poise(parent: :catkin)
    provides(:ros_service)

    attribute(:service_name, kind_of: String, name_attribute: true)
    attribute(:options, kind_of: [String, NilClass], default: nil)

    actions(:enable, :disable, :start, :stop, :restart, :reload)
  end

  class Provider < Chef::Provider
    include Poise
    provides(:ros_service)

    def action_enable
      service(:enable)
    end

    def action_disable
      service(:disable)
    end

    def action_start
      service(:start)
    end

    def action_stop
      service(:stop)
    end

    def action_restart
      service(:restart)
    end

    def action_reload
      service(:reload)
    end

    private

    def service(requested_action)
      # Ensure the supervisor cookbook installs all the dependencies.
      include_recipe 'supervisor'

      supervisor_service new_resource.service_name do
        command "#{new_resource.parent.ros_cmd} #{new_resource.service_name} #{new_resource.options}"
        user new_resource.parent.user
        action requested_action
      end
    end

  end
end
