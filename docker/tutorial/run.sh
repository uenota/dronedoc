#!/bin/bash

PROGNAME=$(basename $0)

usage() {
    echo "Usage: $PROGNAME -h | --help"
    echo "Usage: $PROGNAME --imname image_name [--gpu] [--uname user_name]"
    echo
    echo "  -h, --help: Print usage"
    echo "  --imname  : Name of docker image to use"
    echo "  --gpu     : Use GPU mode if set"
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
	--gpu)
	    gpu=true
	    shift 1
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
    uname=developer
    echo "[INFO] --uname not specified. Use \"$uname\" instead."
fi

if [ -z "$gpu" ]; then
    gpu=false
    echo "[INFO] Use No GPU mode"
fi

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

if [ $gpu = "true" ]; then
    docker run --runtime=nvidia -it --rm \
               --volume=$XSOCK:$XSOCK:rw \
               --volume=$XAUTH:$XAUTH:rw \
               --env="XAUTHORITY=${XAUTH}" \
               --env="DISPLAY=$DISPLAY" \
               --env="QT_X11_NO_MITSHM=1" \
               --user=$uname \
               $imname \
               bash
else
    docker run -it --rm \
               --volume=$XSOCK:$XSOCK:rw \
               --volume=$XAUTH:$XAUTH:rw \
               --env="XAUTHORITY=${XAUTH}" \
               --env="DISPLAY=$DISPLAY" \
               --env="QT_X11_NO_MITSHM=1" \
               --user=$uname \
               $imname \
               bash
fi
