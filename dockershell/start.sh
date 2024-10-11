#!/bin/bash

GID=$(stat -c %g /var/run/docker.sock)
echo docker.sock GID=${GID}

if [[ $? -eq 0 ]]; then
  DGID=$(id -g docker)
  if [[ ${DGID} -ne ${GID} ]]; then
    echo update docker group id to ${GID}
    /usr/sbin/groupmod -g ${GID} docker
  fi
fi

/usr/sbin/sshd -D
