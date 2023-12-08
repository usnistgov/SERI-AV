docker run -it \
	-e DISPLAY=${DISPLAY} \
	--privileged \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	--device=/dev/video0:/dev/video0 \
	--runtime=nvidia \
	--gpus all \
	--network host \
	--ipc=host seri-av:latest
