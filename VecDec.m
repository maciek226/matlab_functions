% Copyright Maciej Lacki 2019
% All rights reserved 

function [R,Cr] = VecDec(R1, R2, R3, V1)
%VECDEC     Decomposes a vector along 3 axes
%   VECDEC(R1, R2, R3, V1) R1 to R3 are the principle axis, and V1 is a
%   decomposed vector. The funtion return a vecotr with proportions of the
%   principle axis producing V1
%Vectro Decomposition
%Decompose a vector V1 onto 3 other Vectors, R1, R2, R3. 
%If Impossible returns NaN

M = [
    R1(1) R2(1) R3(1) V1(1); 
    R1(2) R2(2) R3(2) V1(2); 
    R1(3) R2(3) R3(3) V1(3)];

if (det(M(1:3,1:3)) ~= 0)
    C = rref(M);
    
    R = zeros(3);

    R(1,:) = R1 * C(1,4);
    R(2,:) = R2 * C(2,4);
    R(3,:) = R3 * C(3,4);
    Cr = [C(1,4) C(2,4) C(3,4)]';

else
    R = NaN
end

end

