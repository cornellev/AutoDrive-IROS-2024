FROM autodriveecosystem/autodrive_f1tenth_api:2024-iros-practice

WORKDIR /home/autodrive_devkit

SHELL ["/bin/bash", "-c"]

# Set up bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /home/autodrive_devkit/install/setup.bash" >> ~/.bashrc

# Copy the entire directory contents into the container's workspace
COPY . /home/autodrive_devkit/

# Install declared packages from package.xml
RUN source /opt/ros/foxy/setup.bash \
    && cd src \
    && rosdep update --include-eol-distros \
    && rosdep install --from-paths . -i -y --include-eol-distros

# Build the workspace
RUN source /opt/ros/foxy/setup.bash && colcon build

# Expose simulation port
EXPOSE 4567

COPY entrypoint.sh /home/autodrive_devkit/entrypoint.sh

ENTRYPOINT [ "/home/autodrive_devkit/entrypoint.sh" ]
