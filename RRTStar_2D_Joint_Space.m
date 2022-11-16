% RRT* algorithm in 2D with collision avoidance.
% 
% Author: Sai Vemprala
% 
% nodes:    Contains list of all explored nodes. Each node contains its
%           coordinates, cost to reach and its parent.
% 
% Brief description of algorithm: 
% 1. Pick a random node q_rand.
% 2. Find the closest node q_near from explored nodes to branch out from, towards
%    q_rand.
% 3. Steer from q_near towards q_rand: interpolate if node is too far away, reach
%    q_new. Check that obstacle is not hit.
% 4. Update cost of reaching q_new from q_near, treat it as Cmin. For now,
%    q_near acts as the parent node of q_new.
% 5. From the list of 'visited' nodes, check for nearest neighbors with a 
%    given radius, insert in a list q_nearest.
% 6. In all members of q_nearest, check if q_new can be reached from a
%    different parent node with cost lower than Cmin, and without colliding
%    with the obstacle. Select the node that results in the least cost and 
%    update the parent of q_new.
% 7. Add q_new to node list.
% 8. Continue until maximum number of nodes is reached or goal is hit.

clearvars
close all
clc

% define the 2R robot
l = .2; % length of each link
n = 2;  % number of links
% initial_config = [0.349065850398866 0];         % init_config = [20 deg, 0 deg]
% initial_config = [2*pi, 2*pi];         % init_config = [20 deg, 0 deg]
initial_config = [0, 2*pi];         % init_config = [20 deg, 0 deg]

for i = 1:n
    L(i) = Link([0          0      l/2      0       0     0 ]  ,   'standard');
    robot{2*i-1}= SerialLink(L);
    L(i) = Link([0          0      l        0       0     0 ]  ,   'standard');
    robot{2*i} = SerialLink(L);
end


% Limits of my joint space (qlim on my robot)
x_max = 2*pi;
y_max = 2*pi;

% Epsilon is the distance I step forward towards my randomly generated
% point.
EPS = 15*2*pi/1000;

% This is the max number of iterations my algorithm will run for.
numNodes = 3000;        

% Define starting and ending nodes; note that each node has 3 important
% properties: 1) where it is mapped in joint space (coord), 2) cost it took
% to get there (this is the number of nodes I travel until I get to the
% point), and 3) it keeps track of my parent node.
q_start.coord = initial_config;
q_start.cost = 0;
q_start.parent = 0;
% q_goal.coord = [1.178097245096172, 0.071558499331768];       % Goal angles are (67.5 deg, 4.1 deg)
q_goal.coord = [1.4, 6];       % Goal angles are (67.5 deg, 4.1 deg)
q_goal.cost = 0;

nodes(1) = q_start;
figure(1)
axis([0 x_max 0 y_max])

% Define obstacle
% obstacle = [pi,pi/6,2*pi/5,2*pi/5];         % The obstacle is defined as [x, y, width, height]
% How will you plot your obstacle?
    %define the ellipsoid in cartesian space

    x0 = .215;   %x intercept
    y0 = .215;   %y intercept
    a = .1;      %scale factor for x
    b = .1;      %scale factor for y
    [X,Y,Z] = ellipsoid(x0,y0,0,a,b,0);
    % subplot(1,2,1)
    surf(X,Y,Z)
    xlabel('X (m)')
    ylabel('Y (m)')
    xlim([-.4 .4])
    ylim([-.4 .4])
%     view(2)
    robot{4}.name = 'RRT* Demo';
    robot{4}.plot(initial_config)
%     robot{4}.plot(q_goal.coord)
%     robot{4}.teach(initial_config)

figure(2)
% subplot(1,2,2)
points = work2conf(x0, y0, a, b);
scatter(points(:,1),points(:,2),'r.')
ylabel('q2 (radians)')
xlabel('q1 (radians)')
ylim([0,2*pi])
xlim([0,2*pi])
hold on

for i = 1:1:numNodes
    q_rand = [(rand(1)*x_max) (rand(1)*y_max)];
    if i < 25 
        plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])
    end
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    q_new.coord = steer(q_rand, q_near.coord, val, EPS);
    if noCollision_Joints(q_rand, q_near.coord, x0, y0, a, b)
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        % Note that cost is a function of cumulative distance of all of the
        % nodes.
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        
        % Within a radius of r, find all existing nodes
        q_nearest = [];             % Define struct for nearest node.
        r = 60*2*pi/1000;           % Scaling of radius from the provided example to a space appropriate for our joint space.
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if noCollision_Joints(nodes(j).coord, q_new.coord, x0, y0, a, b)...
                    && dist(nodes(j).coord, q_new.coord) <= r
                
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        
        for k = 1:1:length(q_nearest)
            if noCollision_Joints(q_nearest(k).coord, q_new.coord, x0, y0, a, b)...
                    && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                hold on
            end
        end
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % Append to nodes
        nodes = [nodes q_new];
    end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
joint_angles = [q_start.coord(1), q_start.coord(2)];      % This is the array to return my joint angles
while q_end.parent ~= 0
    start = q_end.parent;
    joint_angles = [joint_angles;
                   [q_end.coord(1), q_end.coord(2)]];     % Returns joint angles in order from end to beginning
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    q_end = nodes(start);
end
joint_angles = flip(joint_angles);                        % Flips orders to return joint angles in order from beginning to end.

for i = 1:(length(joint_angles) - 1)
    figure(1)
    robot{4}.plot(joint_angles(i,:))
    hold on
end