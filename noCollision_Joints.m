function no_collision = noCollision_Joints(n2, n1, x0, y0, a, b)
%     A = [n1(1) n1(2)];
%     B = [n2(1) n2(2)];

    % x coordinates = j1s = q1
    % y coordinates = j2s = q2
    j1s = linspace(n2(1),n1(1),11);
    j2s = linspace(n2(2),n1(2),11);
    
    % Check if path from n1 to n2 intersects any of the four edges of the
    % obstacle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Check if the point lies within the collision area.
    for i = 1:length(j1s)
%         for k = 1:length(j2s)
                test(1)  = sign(   ((cos(j1s(i))/10 - x0).^2)./a^2    + ((sin(j1s(i))/10 - y0).^2)./b^2 -1 );
                test(2)  = sign(   ((cos(j1s(i))/5 - x0).^2)./a^2    + ((sin(j1s(i))/5 - y0).^2)./b^2 -1 );
                test(3)  = sign(   ((cos(j1s(i) + j2s(i))/10 + cos(j1s(i))/5 - x0).^2)./a^2    + ((sin(j1s(i) + j2s(i))/10 + sin(j1s(i))/5 - y0).^2)./b^2 -1 );
                test(4)  = sign(   ((cos(j1s(i) + j2s(i))/5 + cos(j1s(i))/5 - x0).^2)./a^2    + ((sin(j1s(i) + j2s(i))/5 + sin(j1s(i))/5 - y0).^2)./b^2 -1 );
%         end
    end

    if any(test < 0)
        no_collision = 0;     % False (0) if there is a collision
    else
        no_collision = 1;     % Else true (1)
    end
end