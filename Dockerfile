##############################################################
# Dockerfile for Ubuntu16.04
##############################################################
FROM ubuntu:16.04

MAINTAINER Li Qingli "qlli@telenav.cn"

# Install basic tools
RUN apt-get update && \
    apt-get install -y wget \
	build-essential -y \
    zip -y \
    tar \
    vim \
	zsh -y \
    cppcheck

# Install Git last version
RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:git-core/ppa
RUN apt-get update && apt-get install -y git

# Softlink GCC 5.4.0 and g++
RUN ln -sv /usr/bin/gcc /usr/local/bin/gcc && \
  ln -sv /usr/bin/g++ /usr/local/bin/g++

# Install ssh
RUN apt-get update && apt-get install -y openssh-server

# Install CMake
RUN apt-get update && apt-get install -y cmake

# Set up mount point and default command
VOLUME /mnt/ivyClient

ENV PATH=/opt:$PATH
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/local/lib64
ENV CC=/usr/local/bin/gcc
ENV CXX=/usr/local/bin/g++

RUN mkdir /var/run/sshd
CMD ['/usr/sbin/sshd', '-D']

ENTRYPOINT ["bash", "-lc"]
