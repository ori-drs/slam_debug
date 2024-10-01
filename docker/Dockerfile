ARG BASE_IMAGE=ubuntu:22.04

##############
# Base image #
##############
FROM ${BASE_IMAGE} AS base

ARG GTSAM_VERSION=4.2.0

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get -y install \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libeigen3-dev \
 && rm -rf /var/lib/apt/lists/*
RUN cd / \
 && git clone --depth 1 --branch ${GTSAM_VERSION} https://github.com/borglab/gtsam.git \
 && cd gtsam \
 && mkdir build \
 && cd build \
 && cmake -DGTSAM_BUILD_UNSTABLE:OPTION=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. \
 && make -j$(nproc) install \
 && cd / \
 && rm -rf gtsam

ENV DEBIAN_FRONTEND=dialog


#####################
# Development image #
#####################
FROM base AS dev

ARG USERNAME="developer"
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get -y install \
    sudo \
 && rm -rf /var/lib/apt/lists/*
RUN addgroup --gid ${GID} ${USERNAME} \
 && adduser --disabled-password --gecos '' --uid ${GID} --gid ${GID} ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

ENV DEBIAN_FRONTEND=dialog

RUN mkdir -p /home/${USERNAME}/git
WORKDIR /home/${USERNAME}/git
USER ${USERNAME}

