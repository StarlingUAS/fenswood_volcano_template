FROM uobflightlabstarling/starling-controller-base:latest

COPY . /ros_ws/src/fenswood_drone_controller

RUN . /ros_ws/install/setup.sh \
    && colcon build

CMD [ "ros2", "launch", "fenswood_drone_controller", "controller.launch.xml" ]
