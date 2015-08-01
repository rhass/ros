require 'spec_helper'

describe 'catkin_package resource' do
  it 'clones the git repository' do
    expect(file('/opt/catkin_ws/src/roslint/.git')).to be_directory
  end

  subject(:installed_package) { command('/opt/catkin_ws/install/env.sh rospack list') }
  it 'builds and installs a specified catkin package with catkin_make' do
    expect(file('/opt/catkin_ws/build')).to be_directory
    expect(file('/opt/catkin_ws/build/rosserial_arduino')).to be_directory
    expect(installed_package.stdout).to match(/^rosserial_arduino \/opt\/catkin_ws\/install\/share\/rosserial_arduino$/)
    expect(installed_package.stdout).not_to match(/^rosserial_windows \/opt\/catkin_ws\/install\/share\/rosserial_windows$/)
    expect(installed_package.stdout).not_to match(/^rosserial_windows \/opt\/catkin_ws\/install\/share\/rosserial$/)
  end

end

