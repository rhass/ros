require 'spec_helper'

describe 'catkin_package resource' do
  it 'clones the git repository' do
    expect(file('/opt/catkin_ws/src/roslint/.git')).to be_directory
  end

  subject(:installed_package) { command('/opt/catkin_ws/devel/env.sh rospack list') }
  it 'builds the catkin package with catkin_make' do
    expect(file('/opt/catkin_ws/build')).to be_directory
    expect(file('/opt/catkin_ws/build/ros_comm')).to be_directory
    expect(installed_package.stdout).to match(/^rospy \/opt\/catkin_ws\/src\/ros_comm\/clients\/rospy$/)
  end

end

