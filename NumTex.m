% Copyright Maciej Lacki 2019
% All rights reserved 

function [out] = NumTex(input,float)
%NUMTEX Converts a numerical arrey into latex code
%   NUMTEX(input,float)where input is an arrey, and float is the desired
%   number of significant figures which is 4 by deafult 
if nargin == 1
    float = 4;
end

out = latex(vpa(sym(input),float));
end

