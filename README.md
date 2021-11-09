# Setup and Running example using docker

#### Build docker and create a container:

- Build docker: `docker build -t mujoco-tendon .`

- Create a container: `./run.sh`

- Run `xhost +` on your machine to allow `X11 forwarding`

#### Access the docker terminal:

- Get the docker name: `docker container ls`

- and check for the container name column `docker exec -it CONTAINER_NAME /bin/bash`

- Run the example: `python ant_test.py`

# Running Roboy example

- Inside the docker container `cd ~/.mujoco && ./mujoco200/bin/simulate /src/roboy_model/model.xml`
