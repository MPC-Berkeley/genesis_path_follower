clear all; close all; clc;
load('/home/nkapania/Desktop/centralized_three_vehicle_comparison_matched_v10.mat')

%first, calculate s
N             = length(x_traj1(1,:));
m = 5; %number of points to trim at the end
world.roadE   = x_traj1(1,1:end-m)';
world.roadN   = x_traj1(2,1:end-m)';
world.roadPsi = x_traj1(3,1:end-m)' - pi / 2; %accounts for difference in psi
world.s       = zeros(N-m,1);
world.road_IC = [world.roadE(1) world.roadN(1) world.roadPsi(1)];
world.isOpen = 1;

for i = 2:N-m
    world.s(i) = world.s(i-1) + norm([world.roadE(i) - world.roadE(i-1); world.roadN(i) - world.roadN(i-1)]);
end

vp.Ux = x_traj1(4,1:N-m)';
vp.Ax = u_traj1(1,1:N-m)';
vp.s  = world.s;

cmd.delta = u_traj1(2,1:N-m)';
cmd.s     = world.s;

save('/home/nkapania/catkin_ws/src/genesis_path_follower/paths/royaTraj1.mat','world')
save('/home/nkapania/catkin_ws/src/genesis_path_follower/paths/royaVP1.mat','vp')
save('/home/nkapania/catkin_ws/src/genesis_path_follower/paths/royaCmd1.mat','cmd')

