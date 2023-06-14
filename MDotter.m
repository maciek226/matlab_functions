% Copyright Maciej Lacki 2019
% All rights reserved 

function[Ang] = MDotter(M)
%   MDOTTER     Calculates angles between vectors represented by rows of
%   matrix M
%   MDOTTER(M) M is an nx3 matrix. The function outputs a 4x3 matrix with
%   each enetery corepsonding to some combinations of angles
if size(M,2) ~= 3
    disp("Matrix Needs to be n x 3")
    Ang = NaN;
    return
end

Ang = zeros(4,3);
Ang(1,1) = abs(Dotter(M(1,:),M(2,:)));
Ang(1,2) = abs(Dotter(M(1,:),M(3,:)));
Ang(1,3) = abs(Dotter(M(2,:),M(3,:)));

Ang(2,1) = abs(Dotter(-M(1,:),M(2,:)));
Ang(2,2) = abs(Dotter(-M(1,:),M(3,:)));
Ang(2,3) = abs(Dotter(M(2,:),M(3,:)));

Ang(3,1) = abs(Dotter(-M(1,:),-M(2,:)));
Ang(3,2) = abs(Dotter(-M(1,:),M(3,:)));
Ang(3,3) = abs(Dotter(-M(2,:),M(3,:)));

Ang(4,1) = abs(Dotter(M(1,:),-M(2,:)));
Ang(4,2) = abs(Dotter(M(1,:),M(3,:)));
Ang(4,3) = abs(Dotter(-M(2,:),M(3,:)));

end