# F1TENTH

Mașină autonomă la scară 1:10 (F1TENTH / RoboRacer): întâi în simulare, apoi pe hardware real.
Progresie: wall following → follow the gap → pure pursuit → particle filter → MPC.

## Setup (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
sudo apt install -y python3-venv python3-pip libxcb-cursor0 libxcb-icccm4 libxcb-keysyms1 \
  libxcb-shape0 libxcb-randr0 libxcb-render-util0 libxcb-xinerama0 libxkbcommon-x11-0 \
  libgl1 libegl1 libopengl0 libgl1-mesa-dri

cd ~/f1tenth
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
git clone -b dev-jazzy https://github.com/f1tenth/f1tenth_gym_ros.git src/f1tenth_gym_ros
git clone -b dev-jax https://github.com/f1tenth/f1tenth_gym.git src/f1tenth_gym_ros/f1tenth_gym
pip install -e src/f1tenth_gym_ros/f1tenth_gym

source /opt/ros/jazzy/setup.bash
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build   # fără --symlink-install: altfel gym_bridge pornește cu Python-ul sistemului
```

## Rulare simulator

```bash
cd ~/f1tenth
source .venv/bin/activate && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Topic-uri: `/scan` (LaserScan), `/ego_racecar/odom` (Odometry), `/drive` (AckermannDriveStamped).
