% Copyright Maciej Lacki 2018
% All rights reserved 

function [Ang] = Dotter(V1, V2)
%DOTTER     Calulates angle between two vectors (radians)
%   DOTTER(Vec_1, Vec_2) Calculates dot produce between the two vectors and
%   divides it by their notm and takes the acos of the result 
Ang = acos(dot(V1,V2)/norm(norm(V1)*norm(V2)));
end

