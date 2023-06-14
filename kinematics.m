% Copyright Maciej Lacki 2019
% All rights reserved 

function [error, J, Q1, Q2, Q3] = kinematics(Position, Eout)
%   KINEMATICS  Solves the kinemtaics of Novint Falcon device 
%   KINEMTICS(Position) outputs error (0 if succesful), Jacobean J, and 
%   joint angles Q1, Q2, Q3, all of which are vectors with 3 entereis; one for each leg
%   KINEMATICS(Position, Eout) if Eout is 1, the errors will be displayed.
%   Any other value surpesses the errors. 

if nargin == 1
    Eout = 0;
end
error = 0;
%Values obtained from one of the papers. ro Values are the angles from
%World coordinate system to each leg - must be 120* apart, in any
%combination

a = 60e-3; b = 102.5e-3; c = 14.43e-3; d = 11.25e-3; e = d; f = 25e-3; g = 27.9e-3; r = 36.6e-3; s = 23.09e-3;
ro = [0 2*pi/3 4*pi/3 ];

vec = zeros(3);
for cc = 1:3
    %Based on Position of the end effecotr, end point for each leg is
    %calculated by means of a rotation matrix
    vec(cc,:) = [ cos( ro(cc) ) sin( ro(cc) ) 0; -sin( ro(cc) ) cos( ro(cc) ) 0; 0 0 1 ] * transpose(Position) + [ -r; -s; 0 ];
end 
% Take the final position of each point and 
u = vec(:,1);
v = vec(:,2);
w = vec(:,3);

Q3 = zeros(3,1);
Q2 = zeros(3,1);
Q1 = zeros(3,1);
Q1temp = zeros(3,2);
Q2temp = zeros(3,2);

%Finding Solutions to each leg 
for cc = 1:3
    % Q3 exclusivly controls position in one of the directions, and can be
    % calculated imidietly
    c3 = (v(cc) + f) / b; s3 = sqrt( 1 - c3^2);
    if isreal(s3) && isreal(c3)
        Q3(cc) = atan2( s3 , c3);
    else
        if Eout == 1
            disp('The Angle Cannot be Chosen')
        end
        error = 1;
        J = 0;
        Q1 = 0;
        Q2 = 0;
        Q3 = 0;
        return
    end
    
    %ASSUMPTION
    %Q3 can not move beyond 45 degrees
    lim3 = 45;
    if Q3(cc) > deg2rad(90+lim3) && Q3(cc) < deg2rad(90-lim3)
        if Eout == 1
            disp('The Angle Cannot be Chosen')
        end
        error = 1;
        J = 0;
        Q1 = 0;
        Q2 = 0;
        Q3 = 0;
        return
    end
    
    l0 = w(cc)^2 + u(cc)^2 + 2*c*u(cc) - 2*a*u(cc) + a^2 + c^2 - d^2 - e^2 - b^2*sin(Q3(cc))^2 - 2*b*e*sin(Q3(cc)) - 2*b*d*sin(Q3(cc)) - 2*d*e - 2*a*c;
    l1 = -4*a*w(cc);
    l2 = w(cc)^2 + u(cc)^2 + 2*c*u(cc) + 2*a*u(cc) + a^2 + c^2 - d^2 - e^2 - b^2*sin(Q3(cc))^2 - 2*b*e*sin(Q3(cc)) - 2*b*d*sin(Q3(cc)) - 2*d*e + 2*a*c;
    
    t = roots([l2 l1 l0]);
    
    for m = 1:length(t)
        s1 = ( (2*t(m)) / (1+t(m)^2) ); c1 = ( (1-t(m)^2) / (1+t(m)^2) );
        if isreal(s1) && isreal(c1)
            Q1temp(cc,m) = atan2(s1,c1);
        else
            if Eout == 1
                disp('The Angle Cannot be Chosen')
            end
            error = 1;
            J = 0;
            Q1 = 0;
            Q2 = 0;
            Q3 = 0;
            return
        end
    end

    
    %If the solution for Q1 is valid, the calculations will continue
    if isreal(Q1temp)
        %Based on two values of Q1, calculate two values of Q2
        
        for m = 1 : 2 
            c2 = (u(cc)-a*cos(Q1temp(cc,m))+c) / (d + e + b*sin(Q3(cc))); 
            s2 = sqrt( 1 - c2^2);
            
            if isreal(s2) && isreal(c2)
                Q2temp(cc,m) = atan2( s2 , c2);
            else
                if Eout == 1
                    disp('The Angle Cannot be Chosen')
                end
                error = 1;
                J = 0;
                Q1 = 0;
                Q2 = 0;
                Q3 = 0;
                return
            end
            
            
            
        end

        %ASSUMPTION
        %Angles Q1 and Q2 are selected for each leg seperetly. The larger
        %value of Q2 is always used, along with it's corespoding Q1 value
        lim1 = -20;
        lim2 = 110; 

        if Q2temp(cc,1) > Q2temp(cc,2) && Q1temp(cc,1) > deg2rad(lim1) && Q2temp(cc,1) < deg2rad(lim2)
            Q1(cc) = Q1temp(cc,1);
            Q2(cc) = Q2temp(cc,1);
        elseif Q2temp(cc,2) > Q2temp(cc,1) && Q1temp(cc,2) > deg2rad(lim1) && Q2temp(cc,1) < deg2rad(lim2)
            Q1(cc) = Q1temp(cc,2);
            Q2(cc) = Q2temp(cc,2);
        elseif Q2temp(cc,2) == Q2temp(cc,1) && Q1temp(cc,2) > deg2rad(lim1) && Q2temp(cc,1) < deg2rad(lim2)
            Q1(cc) = Q1temp(cc,2);
            Q2(cc) = Q2temp(cc,2);
        else
            if Eout == 1
                disp('The Angle Cannot be Chosen')
            end
            error = 1;
            J = 0;
            Q1 = 0;
            Q2 = 0;
            Q3 = 0;
            return
        end

    else
        if Eout == 1
            disp('Imposible position')
        end
        error = 1;
        J = 0;
        Q1 = 0;
        Q2 = 0;
        Q3 = 0;
        return
    end
end


JF = zeros(3);
ji = zeros(1,3);

%Based on equations from somewhere, Jacobiean component matrecies are
%calculated. Each row is for one leg, each column is calculated below.

for cc = 1:3
    JF(cc,1) = cos(Q2(cc)) * sin(Q3(cc)) * cos(ro(cc)) - cos(Q3(cc)) * sin(ro(cc));
    JF(cc,2) = cos(Q3(cc)) * cos(ro(cc)) + cos(Q2(cc)) * sin(Q3(cc)) * sin(ro(cc));
    JF(cc,3) = sin(Q2(cc)) * sin(Q3(cc));
    ji(cc) = a * sin(Q2(cc) - Q1(cc)) * sin(Q3(cc));
end

JI = diag(ji);
J = JI\JF;

end
