% Copyright Maciej Lacki 2019
% All rights reserved 

function [Velocity, Force] = Zoner(J, Tu)
if nargin == 1
    AngVel = eye(3);
    Torque = AngVel;
    mode = 0;
else
    Torque = diag(Tu);
    mode = 1;
end

if mode == 0
    Velocity = zeros(3);
    Force = Velocity ;
    for cc = 1:3
        %Transposing the result to put it in the right format x1 y1 z1....
        Velocity(cc,:) = transpose(J\AngVel(:,cc));
        Force(cc,:) = transpose(transpose(J)*Torque(:,cc));
    end
    %Scaling the vectors, such that one is a unit vecotr, and the rest are
    %proportinally smaller
    A = [norm(Velocity(1,:)), norm(Velocity(2,:)), norm(Velocity(3,:))];
    B = [norm(Force(1,:)), norm(Force(2,:)), norm(Force(3,:))];
    
    Velocity = Velocity / max(A);
    Force = Force / max(B);
    
elseif mode == 1
    Force = zeros(3);
    for cc = 1:3
        %Transposing the result to put it in the right format x1 y1 z1....
        Force(cc,:) = transpose(transpose(J)*Torque(:,cc));
    end
    
    Velocity = NaN;
end


end
