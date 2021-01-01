#!/bin/bash

PROGNAME=$(basename $0)

usage() {
    echo "Usage: $PROGNAME -h | --help"
    echo "Usage: $PROGNAME --imname image_name --uname user_name"
    echo
    echo "  -h, --help: Print usage"
    echo "  --imname   : Name of docker image to use"
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
        --imname)
            imname=$2
            shift 2
            ;;
        --uname)
            uname=$2
            shift 2
            ;;
    esac
done

# checks if all arguments are defined
if [ -z "$imname" ]; then
    usage
    echo
    echo "[ERROR] Docker image name (--imname) is not defined."
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
    $imname \
    bash
