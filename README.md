# Extended Kalman Filter Project

This is my solution for the EKF project for Udacity's
[Self-Driving Car Nanodegree Program](https://www.udacity.com/drive).

## Building and Running

The following instructions assume that you are using a recent Mac. If you are
not, follow [these instructions](https://www.apple.com/shop/buy-mac/macbook-pro).

Install Docker, `cd` to the directory in the repo containing the `run-docker.sh`
script, and run that script. Within the container, build and run the code as follows.

```bash
cd ..
mkdir build
cd build
cmake ../work
make
./ExtendedKF
```

You can then run the [simulator](https://github.com/udacity/self-driving-car-sim/releases/),
and it should connect to the port exposed by the container.
