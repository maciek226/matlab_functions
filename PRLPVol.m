% Copyright Maciej Lacki 2019
% All rights reserved 

function[Vv] = PRLPVol(VZones)
Vv = zeros(1,4);

Vv(1) = abs(det([VZones(1,:); VZones(2,:); VZones(3,:)]));
Vv(2) = abs(det(-[VZones(1,:); VZones(2,:); VZones(3,:)]));
Vv(3) = abs(det(-[VZones(1,:); -VZones(2,:); VZones(3,:)]));
Vv(4) = abs(det([VZones(1,:); -VZones(2,:); VZones(3,:)]));
end

