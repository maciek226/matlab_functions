% Copyright Maciej Lacki 2019
% All rights reserved 

function SimPlot(Data,type)
if nargin == 1
    type = 1;
end
D = Data{1}.Values.Data;

if type == 1
    plot3(D(:,1), D(:,2), D(:,3));
    grid on
    axis equal vis3d
elseif type == 2
    scatter3(D(:,1), D(:,2), D(:,3));
    axis equal vis3d
end

end