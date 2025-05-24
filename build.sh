#!/usr/bin/env bash

help()
{
   echo "Build a Docker image for Capella ROS application."
   echo
   echo "Syntax: buid.sh [-i|v|j|n|h]"
   echo "options:"
   echo "i     Specify name of the image. [default: ritju/capella-ros]"
   echo "v     Specify version of the image. [default: 1.0.0]"
   echo "j     Maximum jobs allowed to be executed simultaneously. [default: the number of logical CPU cores]"
   echo "n     Disable cache when building image."
   echo "h     Print this Help."
   echo
}

IMAGE=ritju/capella-ros
VERSION=1.0.0
STAGE=final
CACHE_FLAG=
JOB_COUNT=$(getconf _NPROCESSORS_ONLN)

# Get the options
while getopts "hni:v:j:" option; do
   case $option in
      h)
         help
         exit;;
      i)
         IMAGE=$OPTARG;;
      v)
         VERSION=$OPTARG;;
      j)
         JOB_COUNT=$OPTARG;;
      n)
        CACHE_FLAG=--no-cache;;
     \?)
         echo "Error: Invalid option"
         exit;;
   esac
done

IFS='.'
read -a ver <<< $VERSION
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)

echo "Building in directory: $DIR"
echo "Image: $IMAGE"
echo "Tags: ${ver[0]}, ${ver[0]}.${ver[1]}, $VERSION, latest"

docker build --pull $CACHE_FLAG \
 --build-arg VERSION="$VERSION" \
 --build-arg JOB_COUNT="$JOB_COUNT" \
 -t "$IMAGE:${ver[0]}" -t "$IMAGE:${ver[0]}.${ver[1]}" -t "$IMAGE:$VERSION" -t "$IMAGE:latest" \
 --target "$STAGE" "$DIR"
