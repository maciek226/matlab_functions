% Copyright Maciej Lacki 2019
% All rights reserved 

function[V] = SPHVol(Vang)
V = zeros(1,4);
R=1;
for rr = 1:4
    s1 = sum(Vang(rr,:)/2);
    E1 = 4*atan( sqrt( tan(s1/2) *tan( (s1-Vang(rr,1)) / 2 ) *tan( (s1-Vang(rr,2)) / 2 ) *tan( (s1-Vang(rr,3)) / 2) ));
    V(rr) = E1*R^3/(3);
end
end

