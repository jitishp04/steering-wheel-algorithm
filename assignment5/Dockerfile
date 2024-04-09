##################################################
# Section 1: Build the application
FROM ubuntu:22.04 as builder
RUN apt-get update && \
    apt-get install -y gnupg && \
    apt-key adv --recv-keys --keyserver keyserver.ubuntu.com 871920D1991BC93C && \
    apt-get update

MAINTAINER Christian Berger christian.berger@gu.se

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y

RUN apt-get install -y --no-install-recommends \
        cmake \
        build-essential

ADD . /opt/sources
WORKDIR /opt/sources

RUN cd /opt/sources && \
    mkdir build && \
    cd build && \
    cmake -S /opt/sources -D CMAKE_BUILD_TYPE=Release .. && \
    make && make test && cp helloworld /tmp

##################################################
# Section 2: Bundle the application.
FROM ubuntu:22.04
RUN apt-get update -y && \
    apt-get install -y gnupg
MAINTAINER Christian Berger christian.berger@gu.se

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y

WORKDIR /opt
COPY --from=builder /tmp/helloworld .
ENTRYPOINT ["/opt/helloworld"]