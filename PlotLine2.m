% Copyright Maciej Lacki 2019
% All rights reserved 

function PlotLine2(P,V,color, width)
if nargin == 3
    width = 2;
end
if nargin == 2
    width = 2;
    color = 'black';
end
if ischar(color) || isstring(color)
    plot([P(1) P(1)+V(1)], [P(2) P(2)+V(2)], color, 'LineWidth', width)
elseif isnumeric(color)
    if max(color) > 1 || min(color) < 0
        disp(color)
        abs(Normalize(color));
        disp("Color Value is incorrect - Values were Normalized")
    end
    
    plot([P(1) P(1)+V(1)], [P(2) P(2)+V(2)], 'color', color, 'LineWidth', width)

end

