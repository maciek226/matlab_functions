% Copyright Maciej Lacki 2019
% All rights reserved 

function[] = PlotArc(P1, P2, color, width, step)
% PLOTARC: plots an arc between two points on an axis. The color width and
% the number of points used to define a line are customizable 
% PLOTARC(P1, P2): Spcifies two points, plots a black lines with width 2
% and a 100 steps 
% PLOTARC(P1, P2, COLOR): 
% PLOTARC(P1, P2, COLOR, WIDTH)
% PLOTARC(P1, P2, COLOR, WIDTH, STEP)
if nargin == 2
    color = 'black';
    width = 2;
elseif nargin == 3
    step = 100;
    width = 2;
elseif nargin == 4
    step = 100;
end

if size(P1,1) ~= 1 || size(P2,1) ~= 1 || size(P1,2) ~= 3 || size(P2,2) ~= 3
    disp("Provided Vectors do not work ")
    return
end

Dir = P1-P2;
V = zeros(step, 3);
for cc = 1:step
    temp = P2+Dir*cc/step;
    V(cc,:) = temp/norm(temp);
end

if isstring(color) || ischar(color)
    plot3(V(:,1), V(:,2), V(:,3), color, 'LineWidth', width)
end

if isnumeric(color)
    if max(color) > 1 || min(color) < 0
        disp(a)
        abs(Normalize(a));
        disp("Color Value is incorrect - Values were Normalized")
    end
    plot3(V(:,1), V(:,2), V(:,3), 'color', color, 'LineWidth', width)
end
end