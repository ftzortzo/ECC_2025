clear
clc

load("Clays_implementation_S (3).mat")

S(1,:) = -S(1,:) ; 
S(3,:) = -S(3,:) ;



tic
%% IF WE CHANGE 0 to 1 WE WILL DO REGULARIZATION. WE NEED TO DICUSS ABOUT IT.
m=parameter_estimation(S,0)
toc