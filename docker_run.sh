#example call: docker_run.sh seed_dev seed_dev_cnt
xhost +local:root
IMAGE_ID=$(docker images -q $1)
docker run --rm -it --name=$2 --net=host --env="QT_X11_NO_MITSHM=1" --env="DISPLAY=$DISPLAY" -e XAUTHORITY=$XAUTHORITY -v $XAUTHORITY:$XAUTHORITY --privileged --volume="/dev:/dev" --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" --volume=./src:/home/user/ros2_ws/src $1
xhost -local:root
