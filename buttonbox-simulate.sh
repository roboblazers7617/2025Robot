#!/bin/sh
./gradlew build
tmux \
    new-session  './gradlew :simulateJava' \; \
    split-window './gradlew :buttonbox-bridge:run'
