# unitree_go-1


Paso a paso para descargar el entorno y las funcionalidades basicas de ros

wsl -l -v

wsl --unregister Ubuntu-20.04

wsl --install -d Ubuntu-20.04

sudo apt-get update

sudo apt-get upgrade

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc


-----------------------------------------------------------------

mkdir catkin_ws

cd catkin_ws

mkdir src

catkin_make

cd

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc

cd catkin_ws/src

# Clonar los repositorios necesarios (si no los tienes ya)
git clone https://github.com/unitreerobotics/unitree_ros_to_real.git
git clone https://github.com/unitreerobotics/unitree_ros.git
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git

# Volver al workspace y compilar con catkin_make
cd ..

catkin_make




rospack find go2_description
