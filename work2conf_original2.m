
clear

%define the 2R robot
l = .2; %length of each link
n = 2;  %number of links

for i = 1:n
    L(i) = Link([0          0      l/2      0       0     0 ]  ,   'standard');
    robot{2*i-1}= SerialLink(L);
    L(i) = Link([0          0      l        0       0     0 ]  ,   'standard');
    robot{2*i} = SerialLink(L);
end
    
%% Transformation
% transform objets in cartesian space to configuration space
       
%define the elipsoid in cartesian space

x0 = .2;    %x intercept
y0 = .2;    %y intercept
a = .1;    %scale factor for x
b = .1;    %scale factor for y

%create a function that can check if the end or c.o.m. of any link runs into the object

%the limits for the joint angles and the points in between the limits that
%we want to sample
j1s = linspace(0,2*pi,250)';
j2s = linspace(0,2*pi,250)';

%the equations for the test objects in the loops below are equatinos for
%elipsoids, with the x and y coordinates exchanged for the 1st and 2nd
%values for the t vector of the foward kinematics of the robot. There are
%four tests, one for the center of mass and end of each joint
tic
points = [];
for i = 1:length(j1s)
    for k = 1:length(j2s)
            test(1)  = sign(   ((cos(j1s(i))/10 - x0).^2)./a^2    + ((sin(j1s(i))/10 - y0).^2)./b^2 -1 );
            test(2)  = sign(   ((cos(j1s(i))/5 - x0).^2)./a^2    + ((sin(j1s(i))/5 - y0).^2)./b^2 -1 );
            test(3)  = sign(   ((cos(j1s(i) + j2s(k))/10 + cos(j1s(i))/5 - x0).^2)./a^2    + ((sin(j1s(i) + j2s(k))/10 + sin(j1s(i))/5 - y0).^2)./b^2 -1 );
            test(4)  = sign(   ((cos(j1s(i) + j2s(k))/5 + cos(j1s(i))/5 - x0).^2)./a^2    + ((sin(j1s(i) + j2s(k))/5 + sin(j1s(i))/5 - y0).^2)./b^2 -1 );
       if any(test<0)
         points(end+1,:) = [j1s(i),j2s(k)];
       end
    end
end
toc

[X,Y,Z] = ellipsoid(x0,y0,0,a,b,0);
subplot(1,2,1)
surf(X,Y,Z)
xlabel('X (m)')
ylabel('Y (m)')
xlim([-.4 .4])
ylim([-.4 .4])
view(2)

subplot(1,2,2)
scatter(points(:,1),points(:,2),'r')
ylabel('q2')
xlabel('q1')
ylim([0,2*pi])
xlim([0,2*pi])



