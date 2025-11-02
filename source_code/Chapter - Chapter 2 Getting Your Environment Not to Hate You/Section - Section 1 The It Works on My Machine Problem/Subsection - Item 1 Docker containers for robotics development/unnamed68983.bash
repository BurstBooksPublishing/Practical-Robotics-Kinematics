# build image
docker build -t myrobot/dev:ros-noetic .

# run with X11 forwarding and a workspace mount
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/ws:/home/rosdev/ws \   # project code visible inside container
  --device=/dev/ttyUSB0 \         # allow serial sensor access
  --cap-add=SYS_NICE \            # permit real-time priority adjustments
  myrobot/dev:ros-noetic