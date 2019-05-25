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

# We need newer cmake features
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
    python3-dev \
    python-pip \
    python3-pip \
    libeigen3-dev \
    libboost-all-dev \
    libgtest-dev \
    doxygen

# install gtest
RUN cd /usr/src/gtest/ && \
    cmake . && \
    make && \
    cp *.a /usr/lib

# Python packages
RUN pip install \
    numpy \
    pytest

# Python3 packages
RUN pip3 install \
    numpy \
    pytest==4.0.0 \
    sphinx \
    recommonmark \
    mkdocs

# PyBind11
RUN cd ~/ \
    && wget -c https://github.com/pybind/pybind11/archive/v2.2.4.tar.gz \
    -O pybind11-v2.2.4.tar.gz \
    && tar -xzvf pybind11-v2.2.4.tar.gz \
    && cd pybind11-2.2.4/ \
    && mkdir build && cd build \
    && cmake .. \
    && make check -j4 \
    && make install

# Export the linker library
RUN ldconfig /usr/local/lib

# set working directory
RUN mkdir /CoreRobotics
WORKDIR "/CoreRobotics"
