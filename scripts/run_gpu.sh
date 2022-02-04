export DISPLAY=$DISPLAY;
export MUJOCO_PROJECT_PATH=<YOUR_PROJECT_PATH>;

docker run \
    --env="DISPLAY=${DISPLAY}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $MUJOCO_PROJECT_PATH:/code \
    -v $MUJOCO_PROJECT_PATH/mujoco-py:/mujoco-py \
    -w /code/src \
    -d \
    --network host \
    --privileged \
    --gpus all \
    --name ros-mujoco-tendon \
    -it ros-mujoco-tendon /bin/bash
