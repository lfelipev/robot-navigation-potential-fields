close all
clear all
clc

% Define the grid 12x12
xf = 0:0.1:12;
yf = 0:0.1:12;
[X Y] = meshgrid(xf, yf);

% Define the goal location and two obstacles positions
q_goal = [10; 10];
obstacle_1 = [3;3];
obstacle_2 = [9;9];

% Quadratic function d²(q, q_goal)
d = sqrt((q_goal(1)-X).^2 + (q_goal(2)-Y).^2);

% Define the parameters
% zeta to scale the effect of the attractive potential
% n as the gain of the repulsive gradient
% Q to allow the robot to ignore obstacles far away from it
zeta = 10;
n = 10;
Q = 17;
d_goal = 12;

% Attractive potential (conic)
U_att = (1/2) .* zeta .* d;

% if d <= d_goal
%     U_att = (1/2) .* zeta .* d.^2;
% else
%     U_att = d_goal .* zeta .* d - (1/2) .* zeta .* d_goal.^2;
% end

% Define the distance to the obstacles 
D_obstacle_1 = sqrt((obstacle_1(1)-X).^2 + (obstacle_1(2)-Y).^2);
D_obstacle_2 = sqrt((obstacle_2(1)-X).^2 + (obstacle_2(2)-Y).^2);
D = zeros(121,121);

% D is the distance to the closest obstacle D(q)
for i = 1:length(D_obstacle_1)
    for j = 1:length(D_obstacle_1)
        if(D_obstacle_1(i,j) <= D_obstacle_2(i,j))
            D(i,j) = D_obstacle_1(i,j);
        else
            D(i,j) = D_obstacle_2(i,j);
        end
    end
end

% The repulsive potential formula
if D <= Q
    U_rep = (1/2) .* n .*  ((1./D) - (1/Q)).^2;
else
    U_rep = 0;
end

% Sum the potential field
mesh(xf, yf, U_att + U_rep)
axis([0 12 0 12 0 200])