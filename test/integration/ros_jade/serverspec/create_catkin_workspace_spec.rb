require 'spec_helper'

describe 'catkin resource' do
  it 'creates the catkin workspace and src directory' do
    expect(file('/opt/catkin_ws')).to be_directory
    expect(file('/opt/catkin_ws/src')).to be_directory
  end

  it 'initializes the catkin workspace' do
    expect(file('/opt/catkin_ws/src/CMakeLists.txt')).to be_symlink
    expect(file('/opt/catkin_ws/src/CMakeLists.txt')).to be_linked_to('/opt/ros/jade/share/catkin/cmake/toplevel.cmake')
  end
end

