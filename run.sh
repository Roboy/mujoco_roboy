export DISPLAY=$DISPLAY;
export MUJOCO_PROJECT_PATH=/home/thobotics/roboy/mujoco/mujoco-tendon;

docker run \
    --env="DISPLAY=${DISPLAY}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $MUJOCO_PROJECT_PATH/src:/src \
    -v $MUJOCO_PROJECT_PATH/mujoco-py:/mujoco-py \
    -w /src \
    -d \
    --network host \
    --privileged \
    --name ros-mujoco-tendon \
    -it ros-mujoco-tendon /bin/bash
