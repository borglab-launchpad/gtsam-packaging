FROM ubuntu:26.04

LABEL maintainer="bernd.pfrommer@gmail.com"
LABEL version="1.0"
LABEL description="This is custom Docker Image for building GTSAM"
# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive
# Update Ubuntu Software repository
RUN apt-get update && apt-get -y install git pristine-tar git-buildpackage dpkg-dev cmake debhelper gnupg libboost-all-dev libeigen3-dev libtbb-dev libcpputest-dev libmetis-dev libcpputest-dev pybind11-dev libspectra-dev libsuitesparse-dev

