FROM camerondevine/corerobotics

# dev tools
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    nano \
    clang-tidy

# python3
RUN apt-get update && \
    apt-get install -y python3-pip

# cpplint
RUN pip3 install cpplint

# set working directory
RUN mkdir /CoreRobotics
WORKDIR "/CoreRobotics"

