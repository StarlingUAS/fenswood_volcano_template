FROM uobflightlabstarling/starling-controller-base:latest

COPY example_controller_python /ros_ws/src/example_controller_python

RUN . /ros_ws/install/setup.sh \
    && colcon build

CMD [ "ros2", "launch", "example_controller_python", "controller.launch.xml" ]
