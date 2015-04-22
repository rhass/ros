require 'spec_helper'

describe 'catkin_package resource' do
  it 'clones the git repository' do
    expect(file('/opt/catkin_ws/src/roslint/.git')).to be_directory
  end

  subject(:catkin_find) { command('. /opt/ros/jade/setup.sh ; catkin_find xv11_laser_driver') }
  it 'builds the catkin package with catkin_make' do
    expect(file('/opt/catkin_ws/build')).to be_directory
    expect(file('/opt/catkin_ws/build/roslint')).to be_directory
    expect(catkin_find.exit_status).to eq(0)
  end
end

