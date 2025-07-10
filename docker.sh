cat docker.sh
#!/bin/bash
xhost + local: &
echo $1
userName=$USER
dockerName=$1
commandline=$2
sudo docker run --rm --network=host -it \
  -e "ACCEPT_EULA=Y" \
  -v "/home/ubuntu/docker/isaac-sim/cache/ov:/root/.cache/ov:rw" \
  -v "/home/ubuntu/docker/isaac-sim/cache/pip:/root/.cache/pip:rw" \
  -v "/home/ubuntu/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw" \
  -v "/home/ubuntu/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw" \
  -v "/home/ubuntu/docker/isaac-sim/config:/root/.nvidia-omniverse/config:rw" \
  -v "/home/ubuntu/docker/isaac-sim/data:/root/.local/share/ov/data:rw" \
  -v "/home/ubuntu/docker/isaac-sim/documents:/root/Documents:rw" \
  -v "/home/ubuntu/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw" \
  -v "/home/ubuntu/docker/isaac-lab/logs:/workspace/isaaclab/logs:rw" \
  --env DISPLAY=$DISPLAY \
  --device=/dev/dri:/dev/dri \
  --device /dev/snd:/dev/snd \
  --volume "/tmp/.X11-unix/:/tmp/.X11-unix:rw" \
  --volume "/home/$userName/:/media/data" \
  --volume "/media/$userName/:/media/usb" \
  --volume "/dev:/dev" \
  --ipc host \
  --cap-add=ALL \
  --privileged \
  --gpus all \
  --entrypoint /bin/bash \
  -p 8899:8899 \
  -p 8211:8211 \
  $dockerName $commandline

sudo shutdown -h now
