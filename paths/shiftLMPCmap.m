clear all; close all; clc;
cd /home/nkapania/catkin_ws/src/genesis_path_follower/paths
load lmpcMap.mat

xDes = 0;
yDes = -85.00;

world.roadE = world.roadE - world.roadE(1)+xDes;
world.roadN = world.roadN - world.roadN(1)+yDes;

road_IC(2) = world.roadE(1);
road_IC(3) = world.roadN(1);

figure;
plot(world.roadE, world.roadN);
grid on; hold on; axis equal; 

save lmpcMap world