# Setup and start Dockerfile

#### Correct path using BRANCH
Look up the BRANCH in branch.txt.

- Build docker: `docker build -t ros-mujoco-tendon .`

#### Start the Docker container
run `./start.sh`
Docker container mujoco_tendon_BRANCH is run

### Running Roboy example

- Please add your repo path inside either `./run.sh` or `./run_simulation.sh` via `<YOUR_PROJECT_PATH>`

- Make sure that you started `roscore` from your host machine.

- Run the simulation `./scripts/run_simulation.sh`

- Play bag file: `rosbag play ./test_data/shoulder_left.bag -r 2`

- Visualize joint state `python3 ./src/visualize_data.py --body_part shoulder_left`

#### Access the docker terminal:

- Create a container `./scripts/run.sh`

- Access docker container `docker exec -it ros-mujoco-tendon /bin/bash`

