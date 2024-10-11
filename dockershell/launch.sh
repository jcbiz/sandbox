#!/bin/bash

if [[ $# -eq 0 ]]; then
  docker-compose -f dockershell.yml -f global.yml up -d
else
  docker-compose -f dockershell.yml -f global.yml $*
fi


