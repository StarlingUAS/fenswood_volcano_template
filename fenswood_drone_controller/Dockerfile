FROM uobflightlabstarling/starling-controller-base:latest

RUN apt-get update
RUN apt-get install -y python3-pip
RUN pip3 install py_trees

COPY . /ros_ws/src/fenswood_drone_controller

RUN . /ros_ws/install/setup.sh \
    && colcon build

CMD [ "ros2", "launch", "fenswood_drone_controller", "controller.launch.xml" ]
