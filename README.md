# shepharding
Extension of Yating Zheng's elastic model

CREDITS:
Ken Hasselmann for argos-python wrapper <https://github.com/KenN7/argos-python>



# Installation guide
This guide assumes a previously clean installation of Ubuntu20.04

## ARGoS

Step 1: Download and compile ARGoS version 59 
(instructions at https://github.com/ilpincy/argos3)

Step 2: Download and compile E-puck plugin 
(instructions at https://github.com/demiurge-project/argos3-epuck)

## Putting it al together

After downloading the repo..

Step 1: Compile ARGoS-Python

``cd shepherding/argos-python``

``mkdir build``

``cd build``

``cmake ..``

``make``

Step 2: Configuration and Run

Edit experimentconfig.sh file to match your paths
Then run an experiment

``cd shepherding/HelloNeighbor``

``./starter -s``

