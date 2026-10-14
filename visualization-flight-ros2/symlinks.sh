#!/bin/bash

# Create symbolic link from ~/rviz_config to rviz_config

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

ln -s $SCRIPTPATH/rviz_config $HOME/rviz_config

