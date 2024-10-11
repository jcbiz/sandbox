# Use the latest Debian image
FROM debian:stable-slim

# Additional packages
COPY *.deb /tmp/

# Add /sbin and /usr/sbin to the PATH
ENV PATH="/sbin:/usr/sbin:${PATH}"

# Update the package list and install OpenSSH server and other components
RUN apt update && \
    apt install -y --no-install-recommends \
      openssh-server \
      sudo iproute2 iputils-ping traceroute net-tools vim ncat tcpdump psmisc && \
    apt install /tmp/docker-ce-cli_27.3.1-1_bookworm_amd64.deb && \
    apt install /tmp/docker-compose-plugin_2.29.7-1_bookworm_amd64.deb && \
    apt autoremove -y && \
    apt clean && \
    rm -f /tmp/*.deb

# Create the necessary directory for SSH
RUN mkdir /var/run/sshd

# Create a new user with password authentication
RUN groupadd docker
RUN useradd -rm -d /home/docker -s /bin/bash -g docker -G sudo -u 1000 docker
RUN echo 'docker:ChangeIt!!!' | chpasswd

# Expose port 22 for SSH
EXPOSE 22

# Start the SSH service
COPY start.sh /bin/
#CMD ["/usr/sbin/sshd", "-D"]
CMD ["/bin/start.sh"]

