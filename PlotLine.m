% Copyright Maciej Lacki 2019
% All rights reserved 

function[] = PlotLine(P, V, color, width)
% PLOTLINE  Plots a line from P to V
%   PLOTLINE(P, V, Color) P is starting point, V is the end point of the
%   line. Color can be specified as a string, or a vector with 3 terms
%   (RGB)
%   PLOTLINE(P, V, Color, Width) Width may be specified to thickens the
%   line
if nargin == 3
    width = 2;
end
if nargin == 2
    width = 2;
    color = 'black';
end

if ischar(color) || isstring(color)
    
    plot3([P(1) P(1)+V(1)], [P(2) P(2)+V(2)], [P(3) P(3)+V(3)], color, 'LineWidth', width)
    
    
elseif isnumeric(color)
    if max(color) > 1 || min(color) < 0
        disp(color)
        abs(Normalize(color));
        disp("Color Value is incorrect - Values were Normalized")
    end
    
    plot3([P(1) P(1)+V(1)], [P(2) P(2)+V(2)], [P(3) P(3)+V(3)], 'color', color, 'LineWidth', width)
    
end