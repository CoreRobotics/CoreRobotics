FROM camerondevine/corerobotics

# dev tools
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    nano \
    clang-tidy \
    clang-format

# we need cmake 3.6
RUN apt purge -y --auto-remove cmake
RUN cd ~/ \
    && wget https://cmake.org/files/v3.14/cmake-3.14.0.tar.gz \
    && tar -xzvf cmake-3.14.0.tar.gz \
    && cd cmake-3.14.0/ \
    && ./bootstrap \
    && make -j4 \
    && make install
RUN cmake --version

# python3
RUN apt-get update && \
    apt-get install -y python3-pip0

# set working directory
RUN mkdir /CoreRobotics
WORKDIR "/CoreRobotics"

