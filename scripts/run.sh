git rev-parse --abbrev-ref HEAD > branch.txt
BRANCH=$(<branch.txt)

docker build -t mujoco_tendon_$BRANCH .

export DISPLAY=$DISPLAY;
<<<<<<< HEAD:setup.sh
export MUJOCO_PROJECT_PATH=~/Roboy/mujoco_tendon_$BRANCH/mujoco_tendon;
=======
export MUJOCO_PROJECT_PATH=<YOUR_PROJECT_PATH>;
>>>>>>> master:scripts/run.sh

docker run \
    --env="DISPLAY=${DISPLAY}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $MUJOCO_PROJECT_PATH:/code \
    -v $MUJOCO_PROJECT_PATH/mujoco-py:/mujoco-py \
    -w /code/src \
    -d \
    --network host \
    --privileged \
<<<<<<< HEAD:setup.sh
    --name mujoco_tendon_$BRANCH \
    -it mujoco_tendon_$BRANCH /bin/bash
=======
    --name ros-mujoco-tendon \
    -it ros-mujoco-tendon /bin/bash
>>>>>>> master:scripts/run.sh
