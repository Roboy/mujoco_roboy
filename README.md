# Build docker:

`docker build -t mujoco-tendon .`

# Create a container:

`./run.sh`

# Access the docker terminal:

# Get the docker name:
`docker container ls`

# and check for the container name column

`docker exec -it CONTAINER_NAME /bin/bash`

# Run the example:

`python ant_test.py`
