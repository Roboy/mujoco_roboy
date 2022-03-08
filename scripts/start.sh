if [ -z "$(docker ps -q -a -f name=mujoco_roboy)" ]
then
echo "Docker not yet set up."
echo "run setup.sh first."
exit
fi


xhost +
docker container start mujoco_roboy
docker exec -it mujoco_roboy /bin/bash
