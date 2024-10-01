## Instructions

Build the container with the following commands:

```bash
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

