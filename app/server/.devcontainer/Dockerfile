FROM node:18

# Install basic development tools
RUN apt update && apt install -y less man-db sudo wget

RUN wget https://go.dev/dl/go1.22.4.linux-amd64.tar.gz
RUN rm -rf /usr/local/go && tar -C /usr/local -xzf go1.22.4.linux-amd64.tar.gz && rm -f go1.22.4.linux-amd64.tar.gz

# Ensure default `node` user has access to `sudo`
ARG USERNAME=node
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set `DEVCONTAINER` environment variable to help with orientation
ENV DEVCONTAINER=true

ENV PATH=$PATH:/usr/local/go/bin