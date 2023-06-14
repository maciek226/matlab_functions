% Copyright Maciej Lacki 2019
% All rights reserved 

classdef D2p < handle
    % D2P Simulates motion of a 2-DOF parallel manipulator 
    % D2P(l1,l2,l3,l4,l5,P) - constructs device with given link lengths l1
    % to l5. P controls wherether the preview of the device is going to be
    % redendred
    %
    % To change the preview settings used:
    % D2P.Preview = 1 to show 0 to make it go away
    % To show the reference vectors set:
    % D2P.PlotForRef for force 
    % D2P.PlotVelRef for velocity 
    %
    % Class includes:
    % D2P.UpdateP(P) - update position to P = [x;y]
    % D2P.PlotDevice() - Plots the deviec
    properties (SetAccess  = immutable)
        l1 = 1;
        l2 = 1;
        l3 = 1;
        l4 = 1; 
        l5 = 2; 
    end
    properties (SetAccess = private, GetAccess = public)
        P = [0;0]
        pp
        J
        Q1
        Q2
        Q3
        Q4
        Q5
        Q6
        Q7
        F1
        A1
        state = 1;
        nearSing = 0;
        error = .0001;
    end
    properties
        Priview = 1
        PlotForRef = 0
        PlotVelRef = 0
    end
    
    
    methods
        %Constructor
        function h = D2p(l1,l2,l3,l4,l5,p)
            if nargin == 6
                h.l1 = l1;
                h.l2 = l2;
                h.l3 = l3;
                h.l4 = l4;
                h.l5 = l5;
                h.Priview = p;
            elseif nargin ~= 0
                error('The input arguments are inssuficent');
            end
            h.P = [0;h.l1];
            InverseKinematics(h);
            Jacobian(h);
        end
        
        function UpdateQ(h,Q1,Q4)
            h.Fkin(Q1,Q4);
            Jacobian(h);
            if h.Priview == 1
                PlotDevice(h);
            end
                
        end
        
        function UpdateP(h,P)
            Boundery(h,P)
            if h.state == 1 
                h.InverseKinematics();
                %InverseKinematics(h);
                Jacobian(h);
                if h.Priview == 1 && h.state == 1
                    PlotDevice(h);
                end
            else 
                h.J = nan(2);
            end

            
        end
        
        function PlotDevice(h)
            if ishandle(h.F1) 
                set(0, 'CurrentFigure', h.F1);
                cla(h.A1)
            else
                h.F1 = figure();
                h.A1 = gca();
            end
            
            scatter(0,0,'black','filled')
            hold on
            % orgin to 1
            x1 = h.l1*cos(h.Q1);
            y1 = h.l1*sin(h.Q1);
            PlotLine2([0,0],[x1,y1]);
            scatter(x1,y1,'filled','black')
            % 1 to end 
            x2 = h.l2*cos(h.Q2);
            y2 = h.l2*sin(h.Q2);
            scatter(x1+x2,y1+y2,'filled','black')
            PlotLine2([x1,y1],[x2,y2]);
            % Orgin of seocond leg
            x3 = h.l5;
            y3 = 0;
            scatter(x3,y3,'filled','black')
            % point 4 
            x4 = h.l4 * cos(h.Q4);
            y4 = h.l4 * sin(h.Q4);
            scatter(x3+x4,y3+y4,'filled','black')
            PlotLine2([x3,y3],[x4,y4]);
            % End 
            x5 = h.l3 * cos(h.Q3);
            y5 = h.l3 * sin(h.Q3);
            scatter(x3+x4+x5,y3+y4+y5,'filled','black')
            PlotLine2([x3+x4,y3+y4],[x5,y5]);
            % Desired poisiton 
            scatter(h.P(1),h.P(2),'red');
            if h.PlotVelRef == 1
                PlotLine2(h.P,h.J(:,1),'green');
                PlotLine2(h.P,h.J(:,2),'green');
            end
            if h.PlotForRef == 1
                R = inv(h.J)';
                PlotLine2(h.P,R(:,1),'red');
                PlotLine2(h.P,R(:,2),'red');
            end
            
            hold off
            axis equal
            xlim([-h.l1, h.l5 + h.l4])
            ylim([-.5, h.l2+h.l3])
            drawnow
            %figure(h.F1);
        end
        
        
        function Tester(h,Q1,Q2,mode)
            if mode == 1
                h.FKIN2(Q1,Q2);
            else 
                h.FKIN1(Q1,Q2);
            end
            
            if all(not(isnan(h.pp)))
                h.P = h.pp;
                h.InverseKinematics;
                h.Jacobian;
                if h.Priview == 1
                    PlotDevice(h);
                end
            else
                h.J = nan(2);
            end
            
            
        end
        
        % Destructor
        function delete(obj)
            if ishandle(obj.F1)
                close(obj.F1)
            end
        end
    end
    
    methods %(Access = private)
        function Boundery(h,P)
            factor = .95;
            d1 = norm(P);
            d2 = norm([P(1)-h.l5, P(2)]);
            
            r1 = h.l1+ h.l2;
            r2 = h.l3 + h.l4;
            
            % inside both radia, and not negative y
            if d1 < r1*factor && d2 < r2*factor &&  P(2) >=0
                h.P(1) = P(1);
                h.P(2) = P(2);
                h.state = 1; 
            else 
                h.P(1) = nan;
                h.P(2) = nan;
                h.state = 0;
            end

        end
        
        function FKIN1(h,Q1,Q2)
            x1 = cos(Q1) * h.l1;
            y1 = sin(Q1) * h.l1;
            
            x2 = x1 + cos(Q1+Q2) * h.l3;
            y2 = y1 + sin(Q1+Q2) * h.l3;
            
            X5 = [h.l5;0];
            
            R1 = norm([x2;y2]-X5);
            R2 = norm([x2;y2]);
            if R1 < h.l4+h.l3 && y2 >=0 && R2 < h.l1 + h.l2
                %Valid
                h.pp = [x2;y2];

            else
               h.pp = nan(2,1);
            end
        end
        
        function FKIN2(h,Q4,Q3)
            x1 = h.l5 + cos(Q4) * h.l4;
            y1 = sin(Q4) * h.l4;
            
            x2 = x1 + cos(Q4+Q3) * h.l3;
            y2 = y1 + sin(Q4+Q3) * h.l3;
            
            X5 = [h.l5;0];
            
            R1 = norm([x2;y2]-X5);
            R2 = norm([x2;y2]);
            if R1 < h.l4+h.l3 && y2 >=0 && R2 < h.l1 + h.l2
                %Valid
                h.pp = [x2;y2];

            else
               h.pp = nan(2,1);
            end
        end
        
        function Fkin(h,Q1,Q4)
            x1 = h.l1 * cos(Q1);
            x4 = h.l5 + h.l4 * cos(Q4);
            
            y1 = h.l1 * sin(Q1);
            y4 = h.l4 * sin(Q4);
            
            dy = y4 - y1;
            dx  = x4 - x1;
            r = sqrt(dx^2 + dy^2);
            
            if r > h.l2 + h.l3
                h.P = [nan;nan];
                return
            end
            
            Q11 = atan(dy/dx);
            Q12 = acos((-h.l3^2 + h.l2^2 + r^2)/(2 * h.l2 * r));
            q2 = Q11 + Q12;
%             q22 = Q11 - Q12;
            
            q3 = acos((h.l1 * cos(Q1) + h.l2 * cos(q2) - h.l5 - h.l4 * cos(Q4))/h.l3);
            h.Q2 = q2;
            h.Q3 = q3;

            
            p(1) = cos(Q1) * h.l1 + cos(h.Q2) * h.l2;
            p(2) = sin(Q1) * h.l1 + sin(h.Q2) * h.l2;

            h.P = p;
            h.Q1 = Q1;
            h.Q4 = Q4;

            h.Q5 = pi - h.Q4; 
            h.Q6 = h.Q2 - h.Q1;
            h.Q7 = h.Q3 - h.Q4;
        end
     
        function InverseKinematics(h)
            x1 = h.P(1); y1 = h.P(2); r1 = norm([x1,y1]);
            x2 = h.l5 - h.P(1); y2 = h.P(2); r2 = norm([x2,y2]);
            
            alpha1 = atan2(y1,x1);
            abeta1 = -((h.l2^2 - h.l1^2 - r1^2)  / (2 * r1 * h.l1));
            if abeta1^2 > 1 
                h.state = 0;
                h.P = nan(2,1);
                return
            end
            beta1 = atan2(sqrt(1 - abeta1^2), abeta1);
            h.Q1 = alpha1 + beta1;
            
            sq2  = (y1 - sin(h.Q1)*h.l1) / (h.l2);
            q2 = atan2(sq2,sqrt(1-sq2^2));
            if abs(h.l1 * cos(h.Q1) + h.l2 * cos(q2) - x1 ) > .0001 
                h.Q2 = atan2(sq2,-sqrt(1-sq2^2));
            else
                h.Q2 = q2;
            end
            
            alpha2 = atan2(y2,x2);
            abeta2 = -( (h.l3^2 - h.l4^2 - r2^2) / (2 * r2 * h.l4) ); %negative?
            if abeta2^2 > 1 
                h.state = 0;
                h.P = nan(2,1);
                return
            end
            beta2 = atan2(sqrt(1 - abeta2^2),abeta2);
            h.Q4 = pi - beta2 - alpha2;
            sq3 = (y2 - sin(h.Q4)*h.l4) / (h.l3);
            
            if sq3^2 > 1 
                h.state = 0;
                h.P = nan(2,1);
                return
            end
            
            q3 = atan2(sq3,sqrt(1-sq3^2));
            
            if abs(h.l5 + h.l4 * cos(h.Q4) + h.l3 * cos(q3) - x1 ) > .0001
                h.Q3 = atan2(sq3,-sqrt(1-sq3^2));
            else
                h.Q3 =  q3;
            end
            
            h.Q5 = pi - h.Q4; 
            h.Q6 = h.Q2 - h.Q1;
            h.Q7 = h.Q3 - h.Q4;
            h.state = 1;
        end
        
        function ForwardKinematics(h,Q1,Q4)
            range = 50;
            grad = .1;
            Pold = h.P;

            [Q1old,Q2old,Q3old,Q4old,Q5old,Q6old,Q7old] = h.InverseKinematics(Pold);
            
            if isnan(Q1old)
                return
            end
            
            delta = [Q1 - Q1old; Q4 - Q4old];
            errorOld = dot(delta,delta)*1.1;
            
            for cc = 1:range
                Jt = h.Jacobian(Q1old,Q2old,Q3old,Q4old,Q5old,Q6old,Q7old);
                
                delta = [Q1 - Q1old; Q4 - Q4old];
                
                dP = (Jt)*delta*grad;
                
                [Q1new,Q2new,Q3new,Q4new,Q5new,Q6new,Q7new] = h.InverseKinematics(Pold+dP);
                err = dot(delta,delta);
                if isnan(Q1new)
                   grad = grad /3;
                else
                    if err < h.error
                        h.state = 1;
                        h.P = Pold+dP;
                        h.Q1 = Q1;
                        h.Q2 = Q2new;
                        h.Q3 = Q3new;
                        h.Q4 = Q4;
                        h.Q5 = Q5new;
                        h.Q6 = Q6new;
                        h.Q7 = Q7new;
                        return
                    end

                    if err < errorOld
                        Pold = Pold+dP;
                        errorOld = err;
                        Q1old = Q1new;
                        Q2old = Q2new;
                        Q3old = Q3new;
                        Q4old = Q4new;
                        Q5old = Q5new;
                        Q6old = Q6new;
                        Q7old = Q7new;
                    elseif err > errorOld
                        grad = grad / 2; 
                    end
                end
            end
        end
%         
%         function [q11, q12, q13, q14, q15, q16, q17] = InverseKinematics(h,p)
%             if nargin == 1
%                 y = h.P(2); x = h.P(1);
% 
%                 % assume only positive theta1 
%                 C1 = (h.l1^2 - h.l2^2 + x^2 + y^2) / (2*h.l1);
%                 q1 = 2 * atan((-y- sqrt(x^2 + y^2 - C1^2))/(-x-C1));
%                 if isnan(q1) || not(isreal(q1))
%                     h.Q1 = nan;
%                     h.state = 0;
%                     return
%                 else 
%                     h.Q1 = q1;
%                     h.state = 1;
%                 end
% 
%                 % assume only positive theta 4
%                 A2 = x - h.l5;
%                 C2 = (h.l4^2 + h.l5^2 - h.l3^2 - 2 *x * h.l5 + x^2 + y^2)/(2*h.l4);
%                 q4 = 2 * atan((-y + sqrt(A2^2 + y^2 - C2^2))/(-A2-C2));
%                 if isnan(q4) || not(isreal(q4))
%                     h.Q4 = nan;
%                     h.state = 0;
%                     return
%                 else
%                     h.Q4 = q4;
%                     h.state = 1;
%                 end
% 
%                 h.Q2 = asin( (y - h.l1 * sin(h.Q1)) / h.l2 );
%                 x3  = (x - h.l5 - h.l4 *cos(h.Q4) ) / h.l3;
%                 y3 = (y - h.l4 * sin(h.Q4))/h.l3;
%                 h.Q3 = atan2(y3,x3);
% 
%                 h.Q5 = pi - h.Q4; 
%                 h.Q6 = h.Q2 - h.Q1;
%                 h.Q7 = h.Q3 - h.Q4;
%                 
%             elseif nargin == 2
%                 y = p(2); x = p(1);
% 
%                 % assume only positive theta1 
%                 C1 = (h.l1^2 - h.l2^2 + x^2 + y^2) / (2*h.l1);
%                 q1 = 2 * atan((-y- sqrt(x^2 + y^2 - C1^2))/(-x-C1));
%                 if isnan(q1) || not(isreal(q1))
%                     q1 = 2 * atan((-y + sqrt(x^2 + y^2 - C1^2))/(-x-C1));
%                     if isnan(q1) || not(isreal(q1))
%                         q11 = nan;
%                         q12 = nan;
%                         q13 = nan;
%                         q14 = nan;
%                         q15 = nan;
%                         q16 = nan;
%                         q17 = nan;
%                         return
%                     end
%                 else 
%                     q11 = q1;
%                 end
% 
%                 % assume only positive theta 4
%                 A2 = x - h.l5;
%                 C2 = (h.l4^2 + h.l5^2 - h.l3^2 - 2 *x * h.l5 + x^2 + y^2)/(2*h.l4);
%                 q4 = 2 * atan((-y + sqrt(A2^2 + y^2 - C2^2))/(-A2-C2));
%                 if isnan(q4) || not(isreal(q4))
%                     q11 = nan;
%                     q12 = nan;
%                     q13 = nan;
%                     q14 = nan;
%                     q15 = nan;
%                     q16 = nan;
%                     q17 = nan;
%                     return
% %                 elseif (pi-abs(q4)) - abs(q1) < .1
% %                     q14 = pi - q1;
% %                     h.nearSing = 1;
%                 else
%                     q14 = q4;
%                     h.nearSing = 0;
%                 end
% 
%                 q12 = asin( (y - h.l1 * sin(q11)) / h.l2 );
%                 x3  = (x - h.l5 - h.l4 *cos(q14) ) / h.l3;
%                 y3 = (y - h.l4 * sin(q14))/h.l3;
%                 q13 = atan2(y3,x3);
% 
%                 q15 = pi - q14; 
%                 q16 = q12 - q11;
%                 q17 = q13 - q14;
%             end
%         end
        
        function j = Jacobian(h,q11,q12,q13,q14,q15,q16,q17)
            if nargin == 1 
                q1 = h.Q1; q2 = h.Q4; q3 = h.Q6; q4 = h.Q7; l1 = h.l1; l2 = h.l4;

                j11 = -(l1*sin(q2 + q4)*sin(q3)) / sin(q1 - q2 + q3 - q4);
                j12 = (l2*sin(q1 + q3)*sin(q4)) / sin(q1 - q2 + q3 - q4);

                j21 = (l1*cos(q2 + q4)*sin(q3)) / sin(q1 - q2 + q3 - q4);
                j22 = -(l2*cos(q1 + q3)*sin(q4)) / sin(q1 - q2 + q3 - q4);

                h.J = [j11 j12; j21 j22];
                j=h.J;
            elseif nargin == 8
                q1 = q11; q2 = q14; q3 = q16; q4 = q17; l1 = h.l1; l2 = h.l4;

                j11 = -(l1*sin(q2 + q4)*sin(q3)) / sin(q1 - q2 + q3 - q4);
                j12 = (l2*sin(q1 + q3)*sin(q4)) / sin(q1 - q2 + q3 - q4);

                j21 = (l1*cos(q2 + q4)*sin(q3)) / sin(q1 - q2 + q3 - q4);
                j22 = -(l2*cos(q1 + q3)*sin(q4)) / sin(q1 - q2 + q3 - q4);

                j = [j11 j12; j21 j22];
            end
                
        end
    end
    
end
