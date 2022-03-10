if [ -z "$(docker ps -q -a -f name=mujoco_roboy)" ]
then
echo "Docker not yet set up."
echo "run setup.sh first."
exit
fi




xhost +
docker container start mujoco_roboy

read -p "start roscore? y/n: " startroscore

if [[ "$startroscore" == "y" ]]; then
    docker exec -it mujoco_roboy /bin/bash -c "source /root/.bashrc && roscore &" -d
fi

docker exec -it mujoco_roboy /bin/bash -c "source /root/.bashrc && python3 /code/src/simulation.py"
