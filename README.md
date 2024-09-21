# AutoDrive-IROS-2024 Competition Code

## Developing

### With DevContainers
There is a devcontainer set up using the `Dockerfile.dev` dockerfile. You should be able to use DevContainers in many major IDEs, like VSCode, JetBrains, etc. If you don't want to use a devcontainer, you can mount a volume and still use your favorite IDE by following Sid's guide below. To run the container on windows, it may be necessary to replace the `DISPLAY` parameter with `host.docker.internal:0` (where 0 is replaced with the location of your local X11 server) to get X11 forwarding working. Additionally, the path to XMing (or whatever server you're using) must be specified by absolute path instead of using the non-existent tmp directory.

For VSCode, there are some minimal extensions installed (mainly for ROS, C++, and Python). You are welcome to install your own--as long as you do not have to rebuild your container, they will persist. You may have to handle your own setup if you're not using VSCode.

### Without DevContainers
1. Run sim locally
2. Run the devkit in the docker container
3. Test if the rviz window connects to the sim window
4. Create a docker volume
5. Kill the docker container and relaunch with the volume
6. Clone the github repo into the volume, everything should work fine from there

### Notes on Docker
If you want to install more ROS packages or something, you'll have to modify *both* `Dockerfile.dev` and `Dockerfile.prod`, then rebuild the container (VSCode should do this automatically for you when it detects changes).

## Building for prod
1. Just run `docker build -t autodrive_autodrive_iros_2024 -f Dockerfile.prod .` to build the container. 
2. You should now be able to run the container using the competition instructions, like so: `docker run --name autodrive_f1tenth_api --rm -it --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodrive_autodrive_iros_2024`.
