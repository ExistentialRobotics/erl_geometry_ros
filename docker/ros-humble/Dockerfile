ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ARG TZ=America/Los_Angeles
ARG LOCALE=en_US.UTF-8
ARG APT_MIRROR=""
ARG ROS_APT_SOURCE_VERSION=1.1.0

ENV TZ=${TZ}
ENV LANG=${LOCALE}
ENV LANGUAGE=${LOCALE}
ENV LC_ALL=${LOCALE}

RUN cp /etc/apt/sources.list /etc/apt/sources.list.backup &&\
    if [ -n "$APT_MIRROR" ]; then \
    sed -i "s|http://archive.ubuntu.com/ubuntu/|$APT_MIRROR|g" /etc/apt/sources.list; \
    fi
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    DEBIAN_FRONTEND=noninteractive apt update && apt install -y locales && locale-gen en_US.UTF-8

RUN DEBIAN_FRONTEND=noninteractive apt update -y && apt install -y \
    software-properties-common \
    curl

RUN add-apt-repository universe
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
RUN dpkg -i /tmp/ros2-apt-source.deb

RUN DEBIAN_FRONTEND=noninteractive apt update -y && apt upgrade -y
RUN DEBIAN_FRONTEND=noninteractive apt install -y \
    ros-humble-desktop-full \
    ros-dev-tools \
    python3-colcon-common-extensions

# clean
RUN apt-get clean  && rm -rf /var/lib/apt/lists/*
RUN if [ -n "$APT_MIRROR" ]; then \
    mv /etc/apt/sources.list.backup /etc/apt/sources.list; \
    fi
