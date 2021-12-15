export DISPLAY=$DISPLAY;
export MUJOCO_PROJECT_PATH=<YOUR_PROJECT_PATH>;

docker run \
    --env="DISPLAY=${DISPLAY}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $MUJOCO_PROJECT_PATH:/code \
    -v $MUJOCO_PROJECT_PATH/mujoco-py:/mujoco-py \
    -w /code/src \
    --network host \
    --privileged \
    ros-mujoco-tendon /bin/bash -c "source /root/.bashrc && python3 /code/src/simulation.py"
