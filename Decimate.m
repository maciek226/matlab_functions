% Copyright Maciej Lacki 2019
% All rights reserved 

function [M] = Decimate(B,F)
% DECIMATE(B,F) - takes a rectengular matrix B and creates a new matrix
% with the longest dimention divded by factor of F
sz = size(B);
if sz(1) > sz(2)
    M(:,:) = B(1:F:end,:);
elseif sz(2) > sz(1)
    M(:,:) = B(:,1:F:end);
else
    error('The matrix cannot be square');
end
end