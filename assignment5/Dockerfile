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

ADD . /opt/sources/assignment5
WORKDIR /opt/sources/assignment5

RUN pwd && \
    echo "Contents of /opt/sources:" && \
    ls -al && \
    cd assignment5 && \
    pwd && \
    echo "Contents of /opt/sources/assignment5:" && \
    ls -al

RUN mkdir -p /opt/sources/assignment5/build && \
    cd /opt/sources/assignment5/build && \
    cmake -D CMAKE_BUILD_TYPE=Release .. && \
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