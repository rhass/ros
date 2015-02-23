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

class Chef
  class Resource::Catkin < Resource
    include Poise(container: true)

    actions(:create, :remove)

    attribute(:workspace, kind_of: String, name_attribute: true)
    attribute(:user, kind_of: String, default: 'root')
  end

  class Provider::Catkin < Provider
    include Poise

    def action_create
      create_workspace
      install_build_essential
      initialize_workspace
    end

    def action_remove
      remove_workspace
    end

    private

    def workspace_src_dir
      @workspace_src_dir = ::File.join(new_resource.workspace, 'src')
    end

    def install_build_essential
      include_recipe 'build-essential'
    end

    def create_workspace
      directory :workspace_src_dir do
        recursive true
        owner new_resource.user
        action :create
      end
    end

    def initialize_workspace
      execute 'catkin_init' do
        command "catkin_init_workspace #{workspace_src_dir}"
        user new_resource.user
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
