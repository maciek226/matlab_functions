% Copyright Maciej Lacki 2019
% All rights reserved 

classdef D2s < handle
    properties (SetAccess  = immutable)
        l1 = 1;
        l2 = 1;
    end
    properties (SetAccess = private, GetAccess = public)
        P = [0;0]
        P1m
        p2m
        J
        Q1
        Q2
        dQ1
        dQ2
        ddQ1
        ddQ2
        F1
        A1
        state = 1;
        error = .0001;
    end
    properties (SetAccess = private, GetAccess = public) % simulation properties
        Q = [0,0];
        dQ = [0,0];
        ddQ = [0,0];
        g = 9.81;
        f = 100;
        m1 = 1;
        m2 = 2;
        Qsim;
        dQsim;
        ddQsim;
        F2
        A2
        mu = 5;
    end
    properties
        animate = 1;
        record = 0;
        Priview = 1
        PlotForRef = 0
        PlotVelRef = 0
        VectorScale = 0
        LabelVector = 0
        PlotSize = 500;
        DeviceLineThickness = 2;
    end
    
    
    methods
        %Constructor
        function h = D2s(l1,l2,p)
            if nargin == 3
                h.l1 = l1;
                h.l2 = l2;
                h.Priview = p;
            elseif nargin ~= 0
                error('The input arguments are inssuficent');
            end
            h.Q1 = .1;
            h.Q2 = .1;
            Jacobian(h);
        end
        
        % Set angles, calculate the angles, and the jacobina, plot the
        % device
        function UpdateQ(h,Q1,Q2)
            p = h.Kinematics(Q1,Q2);
            h.P = p;
            %h.Boundery(p);
            if h.P == p
                h.Q1 = Q1;
                h.Q2 = Q2;
                Jacobian(h);
                if h.Priview == 1
                    PlotDevice(h);
                end
            end

%             Kinematics(h);
            

        end
        
        % Set position get the angles 
        function UpdateP(h,P)
            Boundery(h,P)
            if h.state == 1
                h.InverseKinematics();
                %InverseKinematics(h);
                Jacobian(h);

                if h.Priview == 1
                    PlotDevice(h);
                end
            else
                h.J = nan(2);
            end
            
        end
        
        
        function Simulaiton(h, tin, m, Qi, dQi, g, f, time, anim, framerate)
            if nargin == 8
                h.animate = 1;
                framerate = 30;
            elseif nargin == 9 
                if f > 60
                    framerate = 30;
                end
            end
            h.mu = [0;0];%[.5,.2]';
            h.animate = anim;
            h.dQ(1) = dQi(1); h.dQ(2) = dQi(2);
            h.Q(1) = Qi(1); h.Q(2) = Qi(2);
            
            h.g = g;
            h.f = f;
            h.m1 = m(1);
            h.m2 = m(2);
            
            iterations = time * f;
            dt = 1/f;
            h.Qsim = zeros(iterations,2);
            h.dQsim = zeros(iterations,2);
            h.ddQsim = zeros(iterations,2);
            
            flag = 0;
            dd = 1;
            skip = round(f/framerate,0);
            
            for cc = 1:iterations
                time = [(cc-1)*dt,(cc)*dt];
                tor = tin;
                
                h.ForwardDynamics(tor,time);
                h.dQsim(cc,:) = h.dQ;
                h.Qsim(cc,:) = h.Q;
                
                if flag == 0
                    flag = 1;
                    h.UpdateQ(h.Q(1),h.Q(2));
                    figure(h.F1)
                else
                    h.ddQsim(cc,:) = (h.dQ-h.dQsim(cc-1,:))/dt;
                end
                
                if h.animate == 1
                    if dd == skip
                        h.UpdateQ(h.Q(1),h.Q(2));
                        dd = 1;
                    else
                        dd = dd + 1;
                    end
                end
                
                
            end

            if all(ishandle(h.F2) == true) && all(not(isempty(h.A2))) && all(isvalid(h.A2))
                set(0, 'CurrentFigure', h.F2)
            else
                h.F2 = figure('Name','Results','Units','Normalized','OuterPosition',[.25, (1-.45)/2, .5, .45]);
            end
            
            subplot(3,2,1)
            plot((1:iterations)*dt, h.Qsim(:,1))
            a1 = gca();
            subplot(3,2,2)
            plot((1:iterations)*dt, h.Qsim(:,2))
            a2 = gca();
            
            subplot(3,2,3)
            plot((1:iterations)*dt, h.dQsim(:,1))
            a3 = gca();
            
            subplot(3,2,4)
            plot((1:iterations)*dt, h.dQsim(:,2))
            a4 = gca();
            
            subplot(3,2,5)
            plot((1:iterations)*dt, h.ddQsim(:,1))
            a5 = gca();
            
            subplot(3,2,6)
            plot((1:iterations)*dt, h.ddQsim(:,2))
            a6 = gca();
            
            h.A2 = [a1,a2,a3,a4,a5,a6];
        end
        
        function [torq] = InverseDynamics(h)
            Ai = [1./(h.l1.^2.*h.m1+h.l1.^2.*h.m2-h.l1.^2.*h.m2.*cos(h.Q2).^2), -(h.l2+h.l1.*cos(h.Q2))./(h.l1.^2.*h.l2.*h.m1+h.l1.^2.*h.l2.*h.m2-h.l1.^2.*h.l2.*h.m2.*cos(h.Q2).^2);
                -(h.l2+h.l1.*cos(h.Q2))./(h.l1.^2.*h.l2.*h.m1+h.l1.^2.*h.l2.*h.m2-h.l1.^2.*h.l2.*h.m2.*cos(h.Q2).^2), (h.l1.^2.*h.m1+h.l1.^2.*h.m2+h.l2.^2.*h.m2+h.l1.*h.l2.*h.m2.*cos(h.Q2).*2.0)./(h.l1.^2.*h.l2.^2.*h.m2.^2+h.l1.^2.*h.l2.^2.*h.m1.*h.m2-h.l1.^2.*h.l2.^2.*h.m2.^2.*cos(h.Q2).^2)];
            B = [-h.l1.*h.l2.*h.m2.*h.dQ2.^2.*sin(h.Q2)-h.l1.*h.l2.*h.m2.*h.dQ1.*h.dQ2.*sin(h.Q2).*2.0;-h.l1.*h.l2.*h.m2.*h.dQ1.*h.dQ2.*sin(h.Q2)];
            C = [h.g.*h.l2.*h.m2.*cos(h.Q1+h.Q2)+h.g.*h.l1.*h.m1.*cos(h.Q1)+h.g.*h.l1.*h.m2.*cos(h.Q1);h.g.*h.l2.*h.m2.*cos(h.Q1+h.Q2)];

            torq = Ai/[h.ddQ1; h.ddQ2] + B + C;
        end
        
        % Destructor
        function delete(obj)
            if ishandle(obj.F1)
                close(obj.F1)
            end
        end
        
    end
    
    methods (Access = private)
        function Boundery(h,P)
            factor = 1;
            if norm(P) < factor * (h.l1+h.l2) && norm(P) > (h.l1-h.l2)/factor && P(2) > 0%norm(P) < factor * (h.l1+h.l2) && norm(P) > (h.l1+h.l2)*(1-factor)*3
                h.P(1) = P(1);
                h.P(2) = P(2);
                h.state = 1;
            else
                h.P(1) = nan;
                h.P(2) = nan;
                h.state = 0;
            end
        end
        
        function InverseKinematics(h)
            x = h.P(1); y = h.P(2);
            r = norm(h.P);
            alpha  = atan2(y,x);
            abeta =  -((h.l2^2 - h.l1^2 - r^2)  / (2 * r * h.l1));
            if abeta^2 < 1
                beta = atan2(sqrt(1 - abeta^2),abeta);
            else
                h.P = nan(2,1);
                h.state = 0;
                return
            end
            h.Q1 = alpha+beta;
            sq2  = (y - sin(h.Q1)*h.l1) / (h.l2);
            q2 = atan2(sq2,sqrt(1-sq2^2));
            
            if abs(h.l1 * cos(h.Q1) + h.l2 * cos(q2) - x ) > .0001
                h.Q2 = -h.Q1 + atan2(sq2,-sqrt(1-sq2^2));
            else
                h.Q2 = -h.Q1 + q2;
            end
            h.state = 1;
        end
        
        function p = Kinematics(h,Q1,Q2)
            if nargin == 1
                h.P = [
                    h.l1 * cos(h.Q1) + h.l2 * cos(h.Q2+h.Q1);
                    h.l1 * sin(h.Q1) + h.l2 * sin(h.Q2+h.Q1)];
                p = h.P;
            elseif nargin == 3
                p = [
                    h.l1 * cos(Q1) + h.l2 * cos(Q2+Q1);
                    h.l1 * sin(Q1) + h.l2 * sin(Q2+Q1)];
            end
        end
        
        function j = Jacobian(h,Q1,Q2)
            if nargin == 1 
                h.J = [
                    -h.l2 * sin(h.Q1 + h.Q2) - h.l1 * sin(h.Q1), -h.l2 * sin(h.Q1+h.Q2);
                    h.l2 * cos(h.Q1 + h.Q2) + h.l1 * cos(h.Q1), h.l2 * cos(h.Q1+h.Q2)];

                
            elseif nargin == 3
                j = [
                    -h.l2 * sin(Q1 + Q2) - h.l1 * sin(Q1), -h.l2 * sin(Q1+Q2);
                    h.l2 * cos(Q1 + Q2) + h.l1 * cos(Q1), h.l2 * cos(Q1+Q2)];

            end
        end
        
        function [] = ForwardDynamics(h, tor, time) 
            if all(size(tor) == [1,2])
                tor=tor';
            end
            
            x = [h.Q(1), h.Q(2), h.dQ(1), h.dQ(2)];
            [~,x] = ode45(@(t,x)DeviceDynamics(t,x,h.m1,h.m2,h.l1,h.l2,h.g,tor,h.mu), time , x(end,:)');

            h.Q(:) = x(end,1:2);
            h.dQ(:) = x(end,3:4);

            
            function dt = DeviceDynamics(~,y,m1,m2,l1,l2,g,torq,u)
                mode = 1; %mode 1 - passive actuators 
                
                theta_1 = y(1);
                theta_2 = y(2);
                theta_dot_1 = y(3);
                theta_dot_2 = y(4);
                
                Ai = [1./(l1.^2.*m1+l1.^2.*m2-l1.^2.*m2.*cos(theta_2).^2), -(l2+l1.*cos(theta_2))./(l1.^2.*l2.*m1+l1.^2.*l2.*m2-l1.^2.*l2.*m2.*cos(theta_2).^2);
                      -(l2+l1.*cos(theta_2))./(l1.^2.*l2.*m1+l1.^2.*l2.*m2-l1.^2.*l2.*m2.*cos(theta_2).^2), (l1.^2.*m1+l1.^2.*m2+l2.^2.*m2+l1.*l2.*m2.*cos(theta_2).*2.0)./(l1.^2.*l2.^2.*m2.^2+l1.^2.*l2.^2.*m1.*m2-l1.^2.*l2.^2.*m2.^2.*cos(theta_2).^2)];
                B = [-l1.*l2.*m2.*theta_dot_2.^2.*sin(theta_2)-l1.*l2.*m2.*theta_dot_1.*theta_dot_2.*sin(theta_2).*2.0;-l1.*l2.*m2.*theta_dot_1.*theta_dot_2.*sin(theta_2)];
                C = [g.*l2.*m2.*cos(theta_1+theta_2)+g.*l1.*m1.*cos(theta_1)+g.*l1.*m2.*cos(theta_1);g.*l2.*m2.*cos(theta_1+theta_2)];                

                bt = zeros(2,1);
                % The else condition should be equat to the torque input, 
                if theta_dot_1 ~= 0
                    bt(1) = -abs(torq(1)/theta_dot_1)*theta_dot_1;
                else
                    bt(1) = torq(1);
                end
                
                if theta_dot_2 ~= 0
                    bt(2) = -abs(torq(2)/theta_dot_2)*theta_dot_2;
                else
                    bt(2) = torq(2);
                end
                
                if mode == 0
                    DDQ = Ai*(torq-B-C - [theta_dot_1;theta_dot_2].*u);
                elseif mode == 1
                    DDQ = Ai*(bt -B-C - [theta_dot_1;theta_dot_2].*u);
                end
                
                dt = [y(3),y(4),DDQ(1),DDQ(2)]';
                
            end   
        end
        
        function PlotDevice(h)
            if all(ishandle(h.F1)== true) && not(isempty(h.A1)) && isvalid(h.A1)
                set(0, 'CurrentFigure', h.F1)
                %cla(h.A1)
            else
                h.F1 = figure('MenuBar','none');
                h.F1.OuterPosition = [1920-h.PlotSize,1080-h.PlotSize,h.PlotSize,h.PlotSize];
                h.F1.ToolBar = 'none';
                h.F1.DockControls = 'off';
                h.F1.Resize = 'off';
                h.A1.xticks=([]);
                h.A1.yticks=([]);
            end
            %h.F1.MenuBar = 'none';
            %h.F1.ToolBar = 'none';
            %h.F1.DockControls = 'off';
            %CurrentPos = h.F1.OuterPosition;
            %h.F1.OuterPosition = [CurrentPos(1:2),max(CurrentPos(3:4))*[1,1]];
            
            
            scatter(0,0,'black')
            hold on
            x1 = h.l1*cos(h.Q1);
            y1 = h.l1*sin(h.Q1);
            PlotLine2([0,0],[x1,y1],'k',h.DeviceLineThickness);
            scatter(x1,y1,'filled','black')
            x2 = h.l2*cos(h.Q2+h.Q1);
            y2 = h.l2*sin(h.Q2+h.Q1);
            scatter(x1+x2,y1+y2,'filled','black')
            scatter(h.P(1),h.P(2),'red');
            PlotLine2([x1,y1],[x2,y2],'k',h.DeviceLineThickness);

            lim = h.l1+h.l2;
            xlim([-lim, lim])
            ylim([-lim, lim])
            if h.PlotVelRef == 1
                if h.VectorScale == 0
                    PlotLine2(h.P,h.J(:,1),[0.4660 0.6740 0.1880]);
                    PlotLine2(h.P,h.J(:,2),[0.4660 0.6740 0.1880]);
                else 
                    V1 = h.J(:,1);
                    V2 = h.J(:,2);
                    V1 = V1/norm(V1)*h.VectorScale;
                    V2 = V2/norm(V2)*h.VectorScale;
                    PlotLine2(h.P,V1,[0.4660 0.6740 0.1880]);
                    PlotLine2(h.P,V2,[0.4660 0.6740 0.1880]);
                end
%                 PlotLine2([x1 y1],h.J(:,1),[0.4660 0.6740 0.1880]);
%                 PlotLine2(h.P,h.J(:,2),[0.4660 0.6740 0.1880]);
            end
            if h.PlotForRef == 1
                R = inv(h.J)';
                if h.VectorScale == 0
                    PlotLine2(h.P,R(:,1),[0.6350 0.0780 0.1840]);
                    PlotLine2(h.P,R(:,2),[0.6350 0.0780 0.1840]);

                else
                    V1 = R(:,1);
                    V2 = R(:,2);
                    V1 = V1/norm(V1)*h.VectorScale;
                    V2 = V2/norm(V2)*h.VectorScale;
                    PlotLine2(h.P,V1,[0.6350 0.0780 0.1840]);
                    PlotLine2(h.P,V2,[0.6350 0.0780 0.1840]);
                    if h.LabelVector == 1
                        text((V1(1)+h.P(1))*1.1,(V1(2)+h.P(2))*1,1,'R_1')
                        text((V2(1)+h.P(1))*1.1,(V2(2)+h.P(2))*1,1,'R_2')
                    end
                end

%                 PlotLine2([x1 y1],R(:,2),[0.6350 0.0780 0.1840]);
%                 PlotLine2(h.P,R(:,1),[0.6350 0.0780 0.1840]);
            end

            hold off
            axis equal
            multi = 1.075;
            xlim([-(h.l1 + h.l2)*multi, (h.l1 + h.l2)*multi])
            ylim([-(h.l1 + h.l2)*multi, (h.l1 + h.l2)*multi])
            
            h.A1 = gca();
            h.A1.InnerPosition = [0 0 1 1];
            xticks([])
            yticks([])
            
            set(gca,'box','off')
            set(gca,'XColor','none')
            set(gca,'YColor','none')
            drawnow
        end
    end
    
end

