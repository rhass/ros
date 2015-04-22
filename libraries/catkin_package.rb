#
# Cookbook Name:: ros
# Library:: catkin_package
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
  class Resource::CatkinPackage < Resource
    include Poise(parent: Catkin)

    attribute(:source_uri, kind_of: String, required: true)
    attribute(:revision, kind_of: String, default: 'master')

    actions(:install, :update, :remove)
  end

  class Provider::CatkinPackage < Provider
    include Poise

    def action_install
      source_package
    end

    def action_upgrade
      source_package
    end

    def action_remove
      remove_workspace
    end

    private

    def install_git
      if node['platform_family'] == 'debian'
        package 'git-core'
      else
        package 'git'
      end
    end

    def source_package
      converge_by("syncing package source for #{new_resource.name}") do
        notifying_block do
          install_git

          git ::File.join(new_resource.parent.workspace_src_dir, new_resource.name) do
            repository new_resource.source_uri
            revision new_resource.revision
            user new_resource.parent.user
            group new_resource.parent.user
            action :sync
            notifies :run, "execute[cmake-#{new_resource.name}]"
          end

          execute "cmake-#{new_resource.name}" do
            command  "catkin_make --directory #{new_resource.parent.workspace}"
            user new_resource.parent.user
            env new_resource.parent.ros_env
            action :nothing
          end
        end
      end
    end

    def remove_workspace
      directory ::File.join(new_resource.parent.workspace_src_dir, new_resource.name) do
        recursive true
        action :remove
      end
    end
  end
end
