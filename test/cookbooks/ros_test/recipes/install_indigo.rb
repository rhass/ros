ros 'indigo' do
  action :install
end

catkin '/opt/catkin_ws' do
  action :create
end
