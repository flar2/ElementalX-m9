#!/bin/sh

/data/local/rngd --rng-device=/dev/urandom --random-device=/dev/random --background --feed-interval=10 --fill-watermark=3584 --random-step=128
