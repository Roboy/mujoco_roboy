# Setup and start Dockerfile

#### Correct path using BRANCH
Look up the BRANCH in branch.txt.

Make sure the folder containing this file (mujoco_tendon) is in ~/Roboy/mujoco_tendon_BRANCH

#### Build a Docker container :
run `./setup.sh`
Docker container mujoco_tendon_BRANCH is created and setup 

#### Start the Docker container
run `./start.sh`
Docker container mujoco_tendon_BRANCH is run

#### Simulate an XML
run `./simulate.sh` in the same folder as the XML to be simulated.
Make sure the XML-path inside simulate.sh is correct.
