# ROS environment inside the container: sourced by interactive shells and by `run.sh <command>`.
source /opt/ros/jazzy/setup.bash
if [ -n "${UR5_WS}" ] && [ -f "${UR5_WS}/install/setup.bash" ]; then
  source "${UR5_WS}/install/setup.bash"
fi
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi
export RCUTILS_COLORIZED_OUTPUT=1
