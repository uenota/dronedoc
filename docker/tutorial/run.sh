#!/bin/bash

PROGNAME=$(basename $0)

usage() {
    echo "Usage: $PROGNAME -h | --help"
    echo "Usage: $PROGNAME --image image_name --uname user_name"
    echo
    echo "  -h, --help: Print usage"
    echo "  --image   : Name of docker image to use"
    echo "  --uname   : User name in docker container"
    echo
}

# parse arguments
for OPT in "$@"
do
    case $OPT in
        -h)
            usage
            exit 0
            ;;
        --image)
            image=$2
            shift 2
            ;;
        --uname)
            uname=$2
            shift 2
            ;;
    esac
done

# checks if all arguments are defined
if [ -z "$image" ]; then
    usage
    echo
    echo "[ERROR] Docker image name (--image) is not defined."
    exit 1
fi

if [ -z "$uname" ]; then
    usage
    echo
    echo "[ERROR] Username in docker container (--uname) is not defined."
    exit 1
fi


XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm \
        --volume=$XSOCK:$XSOCK:rw \
        --volume=$XAUTH:$XAUTH:rw \
        --env="XAUTHORITY=${XAUTH}" \
        --env="DISPLAY" \
        --user=$uname \
    $image \
    bash
