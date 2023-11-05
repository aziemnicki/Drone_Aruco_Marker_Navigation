xhost +local:root
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it \
	--gpus all \
	--workdir="/home" \
    	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XAUTHORITY=$XAUTH" \
	--volume="$XAUTH:$XAUTH" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="/home/$SUDO_USER/ARL/shared:/home/shared:Z" \
	--env="NVIDIA_VISIBLE_DEVICES=all" \
    	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--privileged \
	--network=host \
	--name="TELLO_ROS" \
	tello_ros\
	/bin/bash
