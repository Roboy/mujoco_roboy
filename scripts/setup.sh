MUJOCO_PROJECT_PATH=${PWD%/*}

cd $MUJOCO_PROJECT_PATH/Docker
docker build . -t mujoco_roboy

export DISPLAY=$DISPLAY;
export MUJOCO_PROJECT_PATH;

docker run \
    --env="DISPLAY=${DISPLAY}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $MUJOCO_PROJECT_PATH:/code \
    -v $MUJOCO_PROJECT_PATH/mujoco-py:/mujoco-py \
    -w /code/src \
    -d \
    --network host \
    --privileged \
    --name mujoco_roboy \
    -it mujoco_roboy /bin/bash
