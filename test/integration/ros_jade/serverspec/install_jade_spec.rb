require 'spec_helper'

describe 'install_jade' do
  it 'installs ROS jade' do
    expect(package('ros-jade-ros-base')).to be_installed
  end

  it 'adds a symlink to the ros jade setup.sh in /etc/profiles.d' do
    expect(file('/etc/profile.d/jade.sh')).to be_symlink
    expect(file('/etc/profile.d/jade.sh')).to be_linked_to('/opt/ros/jade/setup.sh')
  end

  subject(:repo_policy) { command('apt-cache policy') }
  it 'configures the ROS apt repository' do
    expect(file('/etc/apt/sources.list.d/ros.list')).to be_file
    expect(file('/etc/apt/sources.list.d/ros.list')).to contain(/http:\/\/packages.ros.org\/ros\/ubuntu/)
    expect(repo_policy.stdout).to match(/http:\/\/packages.ros.org\/ros\/ubuntu/)
  end

  it 'sets the value of LC_ALL to "C"' do
    expect(file('/etc/default/locale')).to be_file
    expect(file('/etc/default/locale')).to contain(/LC_ALL=C/)
  end
end

