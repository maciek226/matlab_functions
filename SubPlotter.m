% Copyright Maciej Lacki 2019
% All rights reserved 

function[Hout]  = SubPlotter(m, n, H, T, sz)
%SUBPLOTTER Creates a single plot from multiple other plots
%   SUBPLOTTER(n,n,H) 
if nargin == 3 
    T = 0;
    sz = 0;
elseif nargin == 4
    sz = 0;
end

S = zeros(m,n);
Hout = figure;

if size(sz,2) == 4
    set(Hout,'units','centimeters', 'OuterPosition',sz)
end

for cc = 1:m*n
    S(cc) = subplot(m,n,cc);
    scatter3(0,0,0);
    axis vis3d equal;
    if isstring(T)
        title(T(cc));
    end
    set(gca, 'Projection','perspective','YTickLabel',[], 'XTickLabel',[],'ZTickLabel',[], 'XColor', 'white', 'YColor', 'white', 'ZColor', 'white');
end

for cc = 1:m*n
    copyobj(allchild(get(H(cc),'CurrentAxes')), S(cc));
    set(H(cc), 'visible', 'off');
end
end