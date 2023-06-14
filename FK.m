% Copyright Maciej Lacki 2019
% All rights reserved 

classdef FK < handle
    properties
        Q = zeros(3,3);          %Joint angles of the device
        J  = zeros(3,3);         %Jacobian
        P  = [0 0 .1];           %Position of the end effector
        Pold  = [-1 -1 -1];      %Position of the end effector - previous 
        error = .00001;          %Accaptable error
        Imax = 15;               %Max iterations
        theta  = zeros(1,3);     %Measured Angles at k
        thetaOld  = zeros(1,3);  %Measured Angles at k-1
        thetaI  = zeros(1,3);    %Initial encoder values
        dev = 0;
    end
    properties (Constant = true)
        a = 60e-3; 
        b = 102.5e-3; 
        c = 14.43e-3; 
%         d = 11.25e-3; 
        d = 13e-3;
        e = 13e-3;
%         e = 11.25e-3; 
        f = 25e-3; 
        g = 27.9e-3; 
        r = 36.6e-3; 
        s = 23.09e-3;
        phi = [0 2*pi/3 4*pi/3];
        PPR = 2048*2;
        T = 2*pi / (2048*2); 
        Pw = pi/(2048*2);
    end
    methods
        %%%%%%%%%%%%%%%%%%%%% Constractor
        function obj = FK(dev)
            if nargin == 0
                return
            elseif nargin == 1 && dev == 1
                obj.dev = 1;
            end
        end
        %%%%%%%%%%%%%%%%%%%%% Set Initial Encoder Values 
        function Home(obj,v1)
            obj.thetaI = obj.Enc([v1(1), v1(2), v1(3)]);
            obj.thetaOld = [v1(1),v1(2),v1(3)];
            obj.P = [0, 0 , .1];
        end
        %%%%%%%%%%%%%%%%%%%%
        function UpdateTheta(obj,v1)
            obj.thetaOld = obj.theta;
            obj.theta = obj.Enc([v1(1), v1(2), v1(3)]) - obj.thetaI;
            obj.ForwardKin;
            obj.Jacobian
        end
        %%%%%%%%%%%%%%%%%%%%
        function Jacobian(obj)
            JF = zeros(3);
            ji = zeros(1,3);
            
            for cc = 1:3
                JF(cc,1) = cos(obj.Q(2,cc)) * sin(obj.Q(3,cc)) * cos(obj.phi(cc)) - cos(obj.Q(3,cc)) * sin(obj.phi(cc));
                JF(cc,2) = cos(obj.Q(3,cc)) * cos(obj.phi(cc)) + cos(obj.Q(2,cc)) * sin(obj.Q(3,cc)) * sin(obj.phi(cc));
                JF(cc,3) = sin(obj.Q(2,cc)) * sin(obj.Q(3,cc));
                ji(cc) = obj.a * sin(obj.Q(2,cc) - obj.Q(1,cc)) * sin(obj.Q(3,cc));
            end
            
            JI = diag(ji);
            obj.J = JI\JF;
        end
        
        function Jac(obj)
            
            for cc = 1:3
                D = obj.a * sin(obj.Q(1,cc)-obj.Q(2,cc))*sin(obj.Q(3,cc));
                obj.J(cc,1) = (cos(obj.Q(3,cc))*sin(obj.phi(cc)) - cos(obj.phi(cc))*cos(obj.Q(2,cc))*sin(obj.Q(3,cc)))/D;
                obj.J(cc,2) = -(cos(obj.phi(cc))*cos(obj.Q(3,cc)) + cos(obj.Q(2,cc))*sin(obj.phi(cc))*sin(obj.Q(3,cc)))/D;
                obj.J(cc,3) = -sin(obj.Q(2,cc))/(obj.a*sin(obj.Q(1,cc)-obj.Q(2,cc)));
            end
        end
        %%%%%%%%%%%%%%%%%%%%
        function InverseKin(obj,Pos)
            %Convert Position to leg refence frames
            vec = zeros(3,3);
            vec(1,:) = ...
                [ cos( obj.phi(1) ) sin( obj.phi(1) ) 0; -sin( obj.phi(1) ) cos( obj.phi(1) ) 0; 0 0 1 ] * ...
                [Pos(1);Pos(2);Pos(3)] + [ -obj.r; -obj.s; 0 ];
            vec(2,:) = ...
                [ cos( obj.phi(2) ) sin( obj.phi(2) ) 0; -sin( obj.phi(2) ) cos( obj.phi(2) ) 0; 0 0 1 ] * ...
                [Pos(1);Pos(2);Pos(3)] + [ -obj.r; -obj.s; 0 ];
            vec(3,:) = ...
                [ cos( obj.phi(3) ) sin( obj.phi(3) ) 0; -sin( obj.phi(3) ) cos( obj.phi(3) ) 0; 0 0 1 ] * ...
                [Pos(1);Pos(2);Pos(3)] + [ -obj.r; -obj.s; 0 ];
            
            
            u = vec(:,1);
            v = vec(:,2);
            w = vec(:,3);
            
            obj.Q(3,1) = acos( (v(1) + obj.f) / obj.b);
            obj.Q(3,2) = acos( (v(2) + obj.f) / obj.b);
            obj.Q(3,3) = acos( (v(3) + obj.f) / obj.b);
            
            
            
            l0 = w(1)^2 + u(1)^2 + 2*obj.c*u(1) - 2*obj.a*u(1) + obj.a^2 + obj.c^2 - obj.d^2 - obj.e^2 - obj.b^2*sin(obj.Q(3,1))^2 - 2*obj.b*obj.e*sin(obj.Q(3,1)) - 2*obj.b*obj.d*sin(obj.Q(3,1)) - 2*obj.d*obj.e - 2*obj.a*obj.c;
            l1 = -4*obj.a*w(1);
            l2 = w(1)^2 + u(1)^2 + 2*obj.c*u(1) + 2*obj.a*u(1) + obj.a^2 + obj.c^2 - obj.d^2 - obj.e^2 - obj.b^2*sin(obj.Q(3,1))^2 - 2*obj.b*obj.e*sin(obj.Q(3,1)) - 2*obj.b*obj.d*sin(obj.Q(3,1)) - 2*obj.d*obj.e + 2*obj.a*obj.c;
            t1 = (-l1 - sqrt(l1^2 - 4 * l0*l2))/(2*l2);
            
            l0 = w(2)^2 + u(2)^2 + 2*obj.c*u(2) - 2*obj.a*u(2) + obj.a^2 + obj.c^2 - obj.d^2 - obj.e^2 - obj.b^2*sin(obj.Q(3,2))^2 - 2*obj.b*obj.e*sin(obj.Q(3,2)) - 2*obj.b*obj.d*sin(obj.Q(3,2)) - 2*obj.d*obj.e - 2*obj.a*obj.c;
            l1 = -4*obj.a*w(2);
            l2 = w(2)^2 + u(2)^2 + 2*obj.c*u(2) + 2*obj.a*u(2) + obj.a^2 + obj.c^2 - obj.d^2 - obj.e^2 - obj.b^2*sin(obj.Q(3,2))^2 - 2*obj.b*obj.e*sin(obj.Q(3,2)) - 2*obj.b*obj.d*sin(obj.Q(3,2)) - 2*obj.d*obj.e + 2*obj.a*obj.c;
            t2 = (-l1 - sqrt(l1^2 - 4 * l0*l2))/(2*l2);
            
            l0 = w(3)^2 + u(3)^2 + 2*obj.c*u(3) - 2*obj.a*u(3) + obj.a^2 + obj.c^2 - obj.d^2 - obj.e^2 - obj.b^2*sin(obj.Q(3,3))^2 - 2*obj.b*obj.e*sin(obj.Q(3,3)) - 2*obj.b*obj.d*sin(obj.Q(3,3)) - 2*obj.d*obj.e - 2*obj.a*obj.c;
            l1 = -4*obj.a*w(3);
            l2 = w(3)^2 + u(3)^2 + 2*obj.c*u(3) + 2*obj.a*u(3) + obj.a^2 + obj.c^2 - obj.d^2 - obj.e^2 - obj.b^2*sin(obj.Q(3,3))^2 - 2*obj.b*obj.e*sin(obj.Q(3,3)) - 2*obj.b*obj.d*sin(obj.Q(3,3)) - 2*obj.d*obj.e + 2*obj.a*obj.c;
            t3 = (-l1 - sqrt(l1^2 - 4 * l0*l2))/(2*l2);
           
            obj.Q(1,1) = atan(t1)*2;
            obj.Q(1,2) = atan(t2)*2;
            obj.Q(1,3) = atan(t3)*2;
            
            obj.Q(2,1) = acos( (u(1) - obj.a*cos(obj.Q(1,1)) + obj.c) / (obj.d+obj.e+obj.b*sin(obj.Q(3,1))));
            obj.Q(2,2) = acos( (u(2) - obj.a*cos(obj.Q(1,2)) + obj.c) / (obj.d+obj.e+obj.b*sin(obj.Q(3,2))));
            obj.Q(2,3) = acos( (u(3) - obj.a*cos(obj.Q(1,3)) + obj.c) / (obj.d+obj.e+obj.b*sin(obj.Q(3,3))));
            obj.Jacobian
        end
        %%%%%%%%%%%%%%%%%%%
        function ForwardKin(obj)
            grad = .0015; 
            erOld = 1;
            delta = zeros(3,1);
            Po = obj.P;
            
            for i = 1:obj.Imax
                
                obj.InverseKin(Po);
                obj.Q;
                obj.Jacobian;
                obj.J;
                
                delta(1) = (obj.theta(1) - obj.Q(1,1));
                delta(2) = (obj.theta(2) - obj.Q(1,2));
                delta(3) = (obj.theta(3) - obj.Q(1,3));
               
                dir = (obj.J * delta)'*grad;
                Pn = Po + dir;
                
                er = dot(delta,delta);
                
                if er < obj.error
                    obj.Pold = obj.P;
                    obj.P = Pn;
                    return
                end
                
                if er > erOld
                    grad = grad/2;
                end
                
                erOld = er;
                Po = Pn;
            end
        end
        %%%%%%%%%%%%%%%%%%%
        function val = Enc(obj,v)
            val = v*obj.Pw;
        end
    end
    
end

