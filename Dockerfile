FROM autodriveecosystem/autodrive_f1tenth_api:2024-iros-practice

WORKDIR /home/autodrive_devkit

SHELL ["/bin/bash", "-c"]

# Set up bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /home/autodrive_devkit/install/setup.bash" >> ~/.bashrc

# Copy package.xml so we can install packages declared there
COPY package.xml src/temp/

# Install declared packages
RUN source /opt/ros/foxy/setup.bash \
    && cd src \
    && rosdep update --include-eol-distros  \
    && rosdep install --from-paths . -i -y --include-eol-distros

# Remove package.xml so things sync correctly with devcontainers
RUN rm -r src/temp/

# Build so workspace is ready
RUN source /opt/ros/foxy/setup.bash && colcon build

# Expose sim port
EXPOSE 4567
