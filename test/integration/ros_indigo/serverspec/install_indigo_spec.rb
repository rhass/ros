require 'spec_helper'

describe 'install_indigo' do
  it 'installs ROS indigo' do
    expect(package('ros-indigo-ros-base')).to be_installed
  end

  it 'adds a symlink to the ros indigo setup.sh in /etc/profiles.d' do
    expect(file('/etc/profile.d/indigo.sh')).to be_symlink
    expect(file('/etc/profile.d/indigo.sh')).to be_linked_to('/opt/ros/indigo/setup.sh')
  end

  subject(:repo_policy) { command('apt-cache policy') }
  it 'configures the ROS apt repository' do
    expect(file('/etc/apt/sources.list.d/ros.list')).to be_file
    expect(file('/etc/apt/sources.list.d/ros.list')).to contain(/http:\/\/packages.ros.org\/ros\/ubuntu/)
    expect(repo_policy.stdout).to match(/http:\/\/packages.ros.org\/ros\/ubuntu/)
  end
end

