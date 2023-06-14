% Copyright Maciej Lacki 2019
% All rights reserved 

function [out] = Exsi(in, stp)
%EXSI Expands and Simplifies the equation
%   EXSI(eq) where eq is the equation to be simplified 
%   EXSI(eq,steps) where steps is the number of simiplification steps 
if nargin == 1 
    steps = 1;
else 
    steps = stp;
end

out = simplify(expand(in),'steps', steps);
end

