# FROM camerondevine/corerobotics
FROM ubuntu:latest

# Dev tools
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    software-properties-common \
    g++ \
    nano \
    wget \
    clang-tidy \
    clang-format \
    autotools-dev \
    libicu-dev \
    libbz2-dev

# We need cmake 3.6 features
RUN apt purge -y --auto-remove cmake
RUN cd ~/ \
    && wget https://cmake.org/files/v3.14/cmake-3.14.0.tar.gz \
    && tar -xzvf cmake-3.14.0.tar.gz \
    && cd cmake-3.14.0/ \
    && ./bootstrap \
    && make -j4 \
    && make install
RUN cmake --version

# CoreRobotics deps
RUN apt-get update && \
    apt-get install -y \
    python-dev \
    python-pip \
    python3-pip \
    libeigen3-dev \
    libboost-all-dev \
    libgtest-dev

# install gtest
RUN cd /usr/src/gtest/ && \
    cmake . && \
    make && \
    ln -s *.a /usr/lib

# Python packages
RUN pip install numpy 

# Python3 packages
RUN pip3 install numpy 

# set working directory
RUN mkdir /CoreRobotics
WORKDIR "/CoreRobotics"
