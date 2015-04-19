ros 'indigo' do
  action :install
end

catkin '/opt/catkin_ws' do
  release 'indigo'
  action :create
end
