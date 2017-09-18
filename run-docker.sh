#!/bin/bash

docker run -it -p 4567:4567 -v $(pwd):/work udacity/controls_kit:latest
