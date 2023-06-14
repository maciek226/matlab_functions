% Copyright Maciej Lacki 2019
% All rights reserved 

function [f, Proj, ccStart] = SSPC(position,plane,mode,F)
% SSP - Plots the path on a plane or a sphere, depending on the mode 
%       SSP(Position, Plane, Mode) - Position matrix (time, X, Y, Z) is
%       projected onto the Plane(time, X, Y, Z, X-offset) in Mode = 1 

if mode == 0 || mode == 0.5
    PlaneSize = size(plane);
    PositionSize = size(position);
    if min(PlaneSize) > 4
        Plane = [plane(1,2) plane(1,3) plane(1,4) plane(1,5)];  %Remove the first entry
    else
        Plane = [plane(1,1) plane(1,2) plane(1,3) plane(1,4)];
    end
    if min(PositionSize) > 3
        Ps = [position(:,2), position(:,3), position(:,4)];
    else 
        Ps = position;
    end
    
    
    
    orgin = scatter3(0,0,0);                                %Creates 3D figure 
    hold on
    store = zeros(10,10,3);                                 %Used to generate a plnae 
    color = 'black';
    colorOld = 'black';
    ccOld = 1;                                              
    flag = 0;
    ccStart = 1;
    for ee = 1:10                                           %Finds location of points on the plane 
        for dd = 1:10
            tx = -.075+ee*.015;
            ty = -.075+dd*.015;
            store(ee,dd,:) = [tx,ty,Plane(4)- (Plane(1)*tx + Plane(2)*ty)/Plane(3)];
        end
    end
    
    s = surf(store(:,:,1),store(:,:,2),store(:,:,3),'HandleVisibility','off');       %Plot the plane 
    s.EdgeColor = 'none';                                   %Grey, No edges, transparent 
    s.FaceColor = [0 0 0];
    s.FaceAlpha = .5;
    
    set(orgin,'Visible','off')                              %Remove the inirial plot of 0
    s = size(position);
    Proj = zeros(4,s(1));
    
    for cc = 1:s(1)
        %Project the position perpendicular onto the plane 
        Proj(1:3,cc) = (cross(Plane(1:3),cross(Ps(cc,:)- [0 0 Plane(4)],Plane(1:3)))/norm(Plane(1:3))^2) + [0 0 Plane(4)]; %+Plane(1:3)*Plane(4)
        
        %Find the distance to the plane 
        d = dot(Plane(1:3),Ps(cc,:)-[0 0 Plane(4)])/norm(Plane(1:3));
        Proj(4,cc) = d;
        if d < 0 %If the position is below the plane - projection is black solid
            color = 'black';
        elseif d  > 0 %If the device is not in contact with the plane the line is dashed 
            color = ':black';
        end
        
        %Plotting takes place when the device enters or exits the wall (ie.
        %when the color of the line changes. The first part of the line is
        %not plotted (assumed to be inside of the surface). 
        
        if color(1) ~= colorOld(1) && flag == 1 
            %plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld)
            if colorOld(1) == 'b'
                plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'LineWidth',2);
            end
            %scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc),20, 'black', 'filled')
            ccOld = cc;
            colorOld = color;
        elseif color(1) ~= colorOld(1) && flag == 0
            %Sets the exit point of the device 
            flag = 1;
            ccOld = cc;
            colorOld = ':black';
            ccStart = cc;
        end
        %plot3(Proj(1,cc-1:cc), Proj(2,cc-1:cc), Proj(3,cc-1:cc), color)
    end
    %plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld,'LineWidth',2)
    if colorOld(1) == 'b'
        plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'LineWidth',2);
    end
    %scatter3(Proj(1,ccStart), Proj(2,ccStart), Proj(3,ccStart),20,  'green','filled')
    %scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc),20, 'green')
    
    axis equal vis3d
    lim = .05;
    xlim([-lim lim])
    ylim([-lim lim])
    zlim([.075 .175])
    f = gcf();
    a = gca();
    set(f,'units','pixels', 'OuterPosition',[(1920-1000)/2 (1080-1000)/2 1000 1000])
    %set(f, 'Render','painters');
    set(f,'GraphicsSmoothing','on');
    hold off
    
elseif mode == 1 
    set(0, 'CurrentFigure', F)
    color = 'black';
    colorOld = 'black';
    ccOld = 1;                                              
    flag = 0;
    ccStart = 1;
    hold on 
    PlaneSize = size(plane);
    PositionSize = size(position);
    if min(PlaneSize) > 4
        Plane = [plane(1,2) plane(1,3) plane(1,4) plane(1,5)];  %Remove the first entry
    else
        Plane = [plane(1,1) plane(1,2) plane(1,3) plane(1,4)];
    end
    if min(PositionSize) > 3
        Ps = [position(:,2), position(:,3), position(:,4)];
    else 
        Ps = position;
    end
    s = size(position);
    Proj = zeros(4,s(1));
    
    for cc = 1:s(1)
        %Project the position perpendicular onto the plane 
        Proj(1:3,cc) = (cross(Plane(1:3),cross(Ps(cc,:)- [0 0 Plane(4)],Plane(1:3)))/norm(Plane(1:3))^2) + [0 0 Plane(4)]; %+Plane(1:3)*Plane(4)
        
        %Find the distance to the plane 
        d = dot(Plane(1:3),Ps(cc,:)-[0 0 Plane(4)])/norm(Plane(1:3));
        Proj(4,cc) = d;
        if d < 0 %If the position is below the plane - projection is black solid
            color = 'black';
        elseif d  > 0 %If the device is not in contact with the plane the line is dashed 
            color = ':black';
        end
        
        %Plotting takes place when the device enters or exits the wall (ie.
        %when the color of the line changes. The first part of the line is
        %not plotted (assumed to be inside of the surface). 
        
        if color(1) ~= colorOld(1) && flag == 1 
            %plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld)
            if colorOld(1) == 'b'
                plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'LineWidth',2);
            end
            %scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc),20, 'black', 'filled')
            ccOld = cc;
            colorOld = color;
        elseif color(1) ~= colorOld(1) && flag == 0
            %Sets the exit point of the device 
            flag = 1;
            ccOld = cc;
            colorOld = ':black';
            ccStart = cc;
        end
        %plot3(Proj(1,cc-1:cc), Proj(2,cc-1:cc), Proj(3,cc-1:cc), color)
    end
    %plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld,'LineWidth',2)
    if colorOld(1) == 'b'
        plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'LineWidth',2);
    end
    %scatter3(Proj(1,ccStart), Proj(2,ccStart), Proj(3,ccStart),20,  'green','filled')
    %scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc),20, 'green')
    
    axis equal vis3d
    lim = .05;
    xlim([-lim lim])
    ylim([-lim lim])
    zlim([.075 .175])
    f = gcf();
    a = gca();
    set(f,'units','pixels', 'OuterPosition',[(1920-1000)/2 (1080-1000)/2 1000 1000])
    hold off
elseif mode == 2
    colorOld = 'red';
    ccOld = 1;
    flag = 0;
    
    
    Po = [Plane(1,2) Plane(1,3) Plane(1,4)];
    R = Plane(1,5);
    Ps = [position(:,2), position(:,3), position(:,4)];
    
    orgin = scatter3(0,0,0,'HandleVisibility','off');
    hold on
    
    [x,y,z] = sphere;
    s= surf(x*R+Po(1),y*R+Po(2),z*R+Po(3));
    s.EdgeColor = 'none';
    s.FaceColor = [0 0 0];
    s.FaceAlpha = .5;
    
    
    
    
    s = size(position);
    Proj = zeros(4,s(1));
    for cc = 1:s(1)
        Proj(1:3,cc) = (Po - Ps(cc))/norm(Po - Ps(cc)) * R + [Po];
        norm(Po - Ps(cc))
        if norm(Po - Ps(cc)) < R
            color = 'red';
        elseif norm(Po - Ps(cc)) == R
            color = 'black';
        else
            color = '--black';
        end
        
        if color(1) ~= colorOld(1) && flag == 1
            plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld)
            scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc), 'black', 'filled')
            ccOld = cc;
            colorOld = color;
        elseif color(1) ~= colorOld(1) && flag == 0
            flag = 1;
            ccOld = cc;
            colorOld = color;
            ccStart = cc;
        end
    end
    plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld)
    plot3(Ps(:,1),Ps(:,2),Ps(:,3),'blue','LineWidth',1);
    set(orgin,'Visible','off')
    
    axis equal vis3d
    lim = .1;
    xlim([-lim lim])
    ylim([-lim lim])
    
    f = gcf();
    set(f,'units','pixels', 'OuterPosition',[(1920-1000)/2 (1080-1000)/2 1000 1000])
    hold off
end



