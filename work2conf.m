function [points] = work2conf(x0, y0, a, b)
    %% Transformation
    % transform objets in cartesian space to configuration space
    %create a function that can check if the end or c.o.m. of any link runs into the object

    %the limits for the joint angles and the points in between the limits that
    %we want to sample
    j1s = linspace(0,2*pi,250)';
    j2s = linspace(0,2*pi,250)';

    %the equations for the test objects in the loops below are equations for
    %ellipsoids, with the x and y coordinates exchanged for the 1st and 2nd
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
end



