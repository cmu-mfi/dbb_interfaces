FROM ros:noetic-ros-core-focal

# Install necessary packages for adding ROS repository
RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN apt-get install -y ros-noetic-catkin
RUN apt-get install -y ros-noetic-industrial-core
RUN apt-get install -y ros-noetic-std-msgs
RUN apt-get install -y tmux
RUN apt-get install -y vim

# Install dependencies from requirements.txt (for our package)
COPY ./requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src/

# Copy my package from the src directory to the catkin workspace in the container
COPY ./ /root/mqtt_ros
WORKDIR /root/mqtt_ros
RUN pip install -e .

# Setup environment so that ROS can be used in bash shell
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

COPY publisher_start.sh /publisher_start.sh
RUN chmod +x /publisher_start.sh

CMD ["bash -c", "echo \"Starting publisher node\""]
RUN echo "ALL Done"