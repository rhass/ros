require 'spec_helper'

describe 'catkin_package resource' do
  it 'clones the git repository' do
    expect(file('/opt/catkin_ws/src/xv_11_laser_driver/.git')).to be_directory
  end

  it 'builds the catkin package with catkin_make' do
    expect(file('/opt/catkin_ws/build')).to be_directory
    expect(file('/opt/catkin_ws/build/xv_11_laser_driver')).to be_directory
  end
end

