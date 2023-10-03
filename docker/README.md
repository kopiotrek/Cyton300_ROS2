# HumbleCyton

How to run:
1. docker build -t humble_cyton .
2. docker run -it --user ros --network=host --ipc=host -v $PWD/source:/my_source_code -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY --env="QT_X11_NO_MITSHM=1" --device=/dev/dri:/dev/dri my_image



