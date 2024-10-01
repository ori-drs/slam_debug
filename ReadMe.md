## Instructions

For changing the versions of the operating system as well as the branch/tag of GTSAM please modify the corresponding variables inside the `docker/docker-compose.yml` file:

```yaml
args:
  - BASE_IMAGE=ubuntu:22.04
  - GTSAM_VERSION=4.2.0
```

Then build the container with the following commands:

```bash
$ docker compose -f docker/docker-compose.yml build
$ docker compose -f docker/docker-compose.yml up
```

connect to the container with another terminal:

```bash
$ docker exec -it slam_debug bash
```

Inside the container run the following instructions to build and run the test:

```bash
$ cd ~/git/slam_debug/
$ rm -r build/
$ mkdir build/
$ cd build/
$ cmake ..
$ make
$ ./simple_slam
```



## Troubleshooting

In case you get the following error message upon execution

```bash
./simple_slam: error while loading shared libraries: libmetis-gtsam.so: cannot open shared object file: No such file or directory
```

please export the following environment variable:

```bash
$ export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

