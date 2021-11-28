# Setup and start Dockerfile

#### Create Dockerfile

run `docker build -t mujoco_tendon .` from inside the mujoco_tendon folder.

#### Setup the Dockerfile:
run `./setup.sh`

#### Start the Docker container
run `./start.sh`

#### Simulate an XML
run `./simulate.sh` in the same folder as the XML to be simulated.
Make sure the XML-path inside simulate.sh is correct.
