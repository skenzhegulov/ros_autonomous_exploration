FROM ros:kinetic

RUN apt-get update && apt-get install -y python-wstool python-rosdep build-essential ros-kinetic-tf ros-kinetic-move-base-msgs

# add user and make sure her stuff is writable whichever userid is given at runtime
# ref: http://blog.dscpl.com.au/2015/12/random-user-ids-when-running-docker.html
RUN adduser --disabled-password --gid 0 --gecos "ROS user" rosuser
RUN mkdir -p /home/rosuser/.ros/log
RUN mkdir -p /home/rosuser/maps
RUN chown rosuser:root /home/rosuser && chmod -R 0775 /home/rosuser
RUN chown rosuser:root /home/rosuser/maps && chmod -R 0775 /home/rosuser/maps
RUN mkdir -p /home/rosuser/catkin_ws

WORKDIR /home/rosuser/catkin_ws/ 

RUN wstool init src \
	&& wstool merge -t src https://raw.githubusercontent.com/skenzhegulov/ros_autonomous_exploration/master/autonomous_exploration.rosinstall \
    && wstool update -t src \
    && rosdep update \
	&& rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build and install.
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"  

COPY ./entrypoint.sh /home/rosuser/

RUN chmod -R g+rwx /home/rosuser/.ros/log  
RUN chgrp -R root /home/rosuser/.ros/log 

RUN chown rosuser:root /home/rosuser/.ros/log && chmod -R 0775 /home/rosuser/.ros/log

#USER rosuser
#WORKDIR /home/rosuser
ENV HOME /home/rosuser

ENTRYPOINT ["/home/rosuser/entrypoint.sh"]
#CMD ["/bin/bash"]
