% Copyright Maciej Lacki 2019
% All rights reserved 

function[NVec] = Normalize(Vec)
%NORMALIZE  creates a unit vector form Vec
%   NORMALIZE(Vec) Divides the vector by its norm
try
    NVec = Vec/norm(Vec);
catch
    disp("Vector cannot be Normalized")
    if norm(Vec)
        disp("Vector Magnitude is 0")
    end
    NVec = NaN
end

