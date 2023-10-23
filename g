#!/bin/bash

tmux kill-window -t 0

cd flowpilot
git pull
git lfs pull

./launch_flowpilot_full.sh
