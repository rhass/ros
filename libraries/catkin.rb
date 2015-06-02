#
# Cookbook Name:: ros
# Library:: catkin
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

class Chef
  class Resource::Catkin < Resource
    include Poise(container: true)

    actions(:create, :remove)

    attribute(:user, kind_of: String, default: lazy { node['current_user'] })
    attribute(:release, kind_of: String, required: true)
    attribute(:workspace, kind_of: String, name_attribute: true)
    attribute(:workspace_src_dir, kind_of: String, default: lazy { ::File.join(self.workspace, 'src') })
    attribute(:ros_path, kind_of: String, default: lazy { ::File.join('/opt/ros', self.release) })
  end

  class Provider::Catkin < Provider
    include Poise

    def action_create
      install_build_essential
      create_workspace
      initialize_workspace
    end

    def action_remove
      remove_workspace
    end

    private

    def install_build_essential
      include_recipe 'build-essential'
    end

    def create_workspace
      directory new_resource.workspace_src_dir do
        recursive true
        owner new_resource.user
        action :create
      end
    end

    def initialize_workspace
      execute 'catkin_init' do
        command "#{new_resource.ros_path}/env.sh catkin_init_workspace"
        cwd new_resource.workspace_src_dir
        user new_resource.user
        creates ::File.join(new_resource.workspace_src_dir, 'CMakeLists.txt')
      end
    end

    def remove_workspace
      directory new_resource.workspace do
        recursive true
        action :delete
      end
    end

  end
end
