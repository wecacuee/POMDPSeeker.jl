FROM ros-melodic-bionic-nvidia

ENV JULIA_VERSION 1.0.5
RUN apt-get update && \
  apt-get install -y stow curl tar python3-pip python3-opencv && \
  rm -rf /var/lib/apt/lists/*
RUN mkdir -p /usr/local/stow/ && \
  curl https://julialang-s3.julialang.org/bin/linux/x64/1.0/julia-$JULIA_VERSION-linux-$(uname -m).tar.gz -o - | \
  tar -C /usr/local/stow/ -xzvf - && \
  cd /usr/local/stow && stow julia-$JULIA_VERSION

ENV HOME /home/root
ENV CODE_DIR $HOME/wrk/POMDPSeeker.jl
RUN mkdir -p $CODE_DIR
COPY . $CODE_DIR
RUN pip3 install --upgrade --no-cache pip setuptools && pip install --no-cache -r $CODE_DIR/requirements.txt
ENV JULIA_PROJECT $CODE_DIR
WORKDIR $CODE_DIR
ENTRYPOINT ["/ros_entrypoint.sh"]
