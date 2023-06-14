% Copyright Maciej Lacki 2019
% All rights reserved 

function [R] = ROT(Directions, angles, rad)
%ROT - outputs a rotation matrix given DIRECTIONS as chacter (X,Y,Z) and angle om rad
if nargin == 2
    rad = 0;
else
    Angles=angles;
end

if not(isstring(Directions)) && not(ischar(Directions))
    error('Direction must be a string or a character')
end

if rad ~= 1
    Angles = deg2rad(angles);
    if Directions(1) == 'x' || Directions(1) == 'X'
        R = [1 0 0; 0 cos(Angles(1)) -sin((Angles(1))); 0 sin((Angles(1))) cos((Angles(1)))];
    elseif Directions(1) == 'y' || Directions(1) == 'Y'
        R = [cos((Angles(1))) 0 sin((Angles(1))); 0 1 0; -sin((Angles(1))) 0 cos((Angles(1)))];
    elseif Directions(1) == 'z' || Directions(1) == 'Z'
        R = [cos(Angles(1)) -sin(Angles(1)) 0; sin(Angles(1)) cos(Angles(1)) 0; 0 0 1];
    end

else
    if Directions(1) == 'x' || Directions(1) == 'X'
        R = [1 0 0; 0 cos(Angles) -sin((Angles)); 0 sin((Angles)) cos((Angles))];
    elseif Directions(1) == 'y' || Directions(1) == 'Y'
        R = [(cos(Angles)) 0 sin((Angles)); 0 1 0; -sin((Angles)) 0 (cos(Angles))];
    elseif Directions(1) == 'z' || Directions(1) == 'Z'
        R = [cos(Angles) -sin(Angles) 0; sin(Angles) cos(Angles) 0; 0 0 1];
    end
end




