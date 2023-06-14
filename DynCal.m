% Copyright Maciej Lacki 2019
% All rights reserved 

function [AngVel, Torque] = DynCal(J, Vel, Frs, mode)
% DYNCAL Calculates the velocity and force, or torque and angular velocity
%   DYNCAL(J, V, F) Calculates the torque and angular velocity of each
%   joint resulting from velocity V, and force F. To function Jacobean J
%   needs to be provided. 
%
%   DYNCAL(J, V, F, mode) MODE 1 inverses the solution. The Jacobean is now
%   used to calcualte the force and velocity resulting from angular
%   counterparts. 
if nargin == 3
    mode = 0;
end
    

if mode == 1
    %Take input Velocity and Force, and Convert them into Torque, and
    %AngVel
    AngVel = J \ Vel;
    Torque = J' * Frs;
    
elseif mode == 0
    %Take input Torque and AngVel, and Convert them into Frs, and Vel
    AngVel = Vel; Torque = Frs;
    
    Vel = J * AngVel;
    Frs = J' \ Torque;
    
    AngVel = Vel; Torque = Frs;
end

end
