FROM humble:dev

RUN apt-key del F42ED6FBAB17C654
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rqt \
    ros-humble-rqt-reconfigure \
  && rm -rf /var/lib/apt/lists/*