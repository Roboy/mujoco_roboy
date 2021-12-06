git rev-parse --abbrev-ref HEAD > branch.txt
BRANCH=$(<branch.txt)

docker build -t mujoco_tendon_$BRANCH .

export DISPLAY=$DISPLAY;
export MUJOCO_PROJECT_PATH=~/Roboy/mujoco_tendon_$BRANCH/mujoco_tendon;

docker run \
    --env="DISPLAY=${DISPLAY}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $MUJOCO_PROJECT_PATH/src:/src \
    -v $MUJOCO_PROJECT_PATH/mujoco-py:/mujoco-py \
    -w /src \
    -d \
    --network host \
    --privileged \
    --name mujoco_tendon_$BRANCH \
    -it mujoco_tendon_$BRANCH /bin/bash
