# ECC_2025

## Hey Alex!

I have drafted the script you can see in main.m. This script generates human-driving–like trajectories so we can test Bayesian Linear Regression (BLR) without the VR data stream. It uses the standard IDM car-following law (gooofle IDM car following model) on a single straight road: vehicles are initialized with reasonable positions/speeds and per-vehicle IDM parameters; at each step we sort vehicles by position to assign leaders, compute the IDM acceleration (including time headway and dynamic desired gap), and integrate to obtain position, speed, and acceleration traces, which are then plotted.

Do not calibrate or analyze IDM here—IDM is just the trajectory generator. Your task is the following. Using the generated trajectories, follow our published paper to estimate the coefficients of Newell’s model (the car-following model we analyze), then apply BLR on those Newell parameters/relations to train and make predictions. In short treat IDM as a data source; use our paper’s methodology to learn Newell’s model from that data; then run BLR and report prediction results (e.g., vary the training window/trigger such as after 60 m traveled and study how data amount affects conservatism).

Note that here we consider only the case where we have human drivers. Thus for the first human driver we have the special case in section III.c of the paper. I think this is the most interesting case. Next we will consider mixed traffic, which will not change things dramatically. 

I hope this helps.


