BRANCH=$(<branch.txt)

xhost +
docker container start mujoco_tendon_$BRANCH
docker exec -it mujoco_tendon_$BRANCH /bin/bash
