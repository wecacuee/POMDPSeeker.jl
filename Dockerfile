FROM ros:melodic-perception-bionic

ENV JULIA_VERSION 1.0.5
RUN apt-get update && apt-get install -y stow curl tar && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /usr/local/stow/ && \
  curl https://julialang-s3.julialang.org/bin/linux/x64/1.0/julia-$JULIA_VERSION-linux-$(uname -m).tar.gz -o - | \
  tar -C /usr/local/stow/ -xzvf - && \
  cd /usr/local/stow && stow julia-$JULIA_VERSION
