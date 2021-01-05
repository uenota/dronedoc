#!/bin/bash
PROGNAME=$(basename $0)

usage() {
    echo "Usage: $PROGNAME -h | --help"
    echo "Usage: $PROGNAME --imname image_name [--uname user_name]"
    echo
    echo "  -h, --help: Print usage"
    echo "  --imname  : Name of docker image to be built"
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
    uname=developer
    echo "[INFO] --uname not specified. Use \"$uname\" instead."
fi

# use uid and gid of the current user
userid=$(id -u)
groupid=$(id -g)

# build docker image
docker image build -t $imname --build-arg username=$uname --build-arg userid=$userid --build-arg groupid=$groupid .

if [[ $? -ne 0 ]]; then
    echo "[ERROR] Build Failed."
    exit 1
fi
echo "[INFO] Image Built (UID: $userid, GID: $groupid, username: $uname)"
