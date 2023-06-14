% Copyright Maciej Lacki 2019
% All rights reserved 

function [C] = LinSimp(B,min)

sz = size(B);
base = zeros(sz);
ii = 1; %Number of eneteris
pp = 1; %Store Previous value 
if sz(1) > sz(2)
    for cc = 1:sz(1)
        if abs(norm(B(cc,:)) - norm(B(pp,:))) > min
            base(ii,:) = B(cc,:);
            ii = ii+1;
            pp = cc;
        end
    end
    
    C = base(1:ii-1,:);

    
elseif sz(2) > sz(1)
    for cc = 1:sz(1)
        if abs(norm(B(:,cc)) - norm(B(:,pp))) > min
            base(:,ii) = B(:,cc);
            ii = ii+1;
            pp = cc;
        end
    end
    
    C = base(:,1:ii-1);

else
    error('The matrix cannot be square');
end


end

