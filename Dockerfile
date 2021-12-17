FROM ubuntu:trusty

RUN apt-get update && \
    apt-get install --assume-yes --no-install-recommends \
        build-essential \
        curl

# https://www.linuxfromscratch.org/lfs/view/stable/chapter08/openssl.html

ARG OPENSSL_VERSION="1.1.1m"
    
RUN cd /tmp && \
    curl -O -k https://www.openssl.org/source/openssl-${OPENSSL_VERSION}.tar.gz && \
    tar -xf openssl-${OPENSSL_VERSION}.tar.gz && \
    cd openssl-${OPENSSL_VERSION} && \
    ls && \
    ./config \
        --openssldir=/usr/local/openssl \
        --prefix=/usr/local/openssl && \
    make && \
    make install

ENV PATH=/usr/local/openssl/bin${PATH:+:$PATH}
ENV LD_LIBRARY_PATH=/usr/local/openssl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

# https://www.linuxfromscratch.org/blfs/view/stable/general/python3.html

RUN apt-get update && \
    apt-get install --assume-yes --no-install-recommends \
        zlib1g-dev

ARG PYTHON_VERSION="3.10.1"

RUN cd /tmp && \
    curl -O -k https://www.python.org/ftp/python/${PYTHON_VERSION}/Python-${PYTHON_VERSION}.tar.xz && \
    tar -xf Python-${PYTHON_VERSION}.tar.xz && \
    cd Python-${PYTHON_VERSION} && \
    ./configure \
        --enable-shared \
        --prefix=/usr/local/python \
        --with-openssl=/usr/local/openssl && \
    make && \
    make install

# https://www.linuxfromscratch.org/blfs/view/stable/x/mesa.html

ENV PATH=/usr/local/python/bin${PATH:+:$PATH}
ENV LD_LIBRARY_PATH=/usr/local/python/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

RUN pip3 install \
        mako \
        meson

RUN apt-get update && \
    apt-get install --assume-yes --no-install-recommends \
        bison \
        flex \
        libx11-dev \
        libxpat1-dev \
        llvm-dev \
        pkg-config

ARG MESA_VERSION="21.3.1"
        
RUN cd /tmp && \
    curl -O -k https://archive.mesa3d.org/mesa-${MESA_VERSION}.tar.xz && \
    tar -xf mesa-${MESA_VERSION}.tar.xz && \
    cd mesa-${MESA_VERSION} && \
    mkdir build && \
    cd build && \
    meson \
        --prefix=/usr/local/mesa

