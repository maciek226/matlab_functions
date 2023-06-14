% Copyright Maciej Lacki 2019
% All rights reserved 

function [f, Proj, Inter] = SSP(position,plane,mode,F)
% SSP - Plots the path on a plane or a sphere, depending on the mode 
%       SSP(Position, Plane, Mode) - Position matrix (time, X, Y, Z) is
%       projected onto the Plane(time, X, Y, Z, X-offset) in Mode = 1 

if mode == 0
    %%
    PlaneSize = size(plane);
    PositionSize = size(position);
    if PlaneSize(2) == 5
        Plane = [plane(1,2) plane(1,3) plane(1,4) plane(1,5)];
    elseif PlaneSize(2) == 4
        Plane = [plane(1,1) plane(1,2) plane(1,3) plane(1,4)];  %Remove the first entry
    else
        Plane = [plane(1,1) plane(1,2) plane(1,3) plane(1,4)];
    end
    if min(PositionSize) > 3
        Ps = [position(:,2), position(:,3), position(:,4)];
    else 
        Ps = position;
    end
    
    Intersection = zeros(1000,1);
    ii = 1;
    
    orgin = scatter3(0,0,0,'HandleVisibility','off');                                %Creates 3D figure 
    hold on
                                   %Used to generate a plnae 
    color = 'black';
    colorOld = 'black';
    ccOld = 1;                                              
    flag = 0;
    
    PlaneXlim = [-.05 .05];
    PlaneYlim = [-.05 .05];
    store = zeros(length(PlaneXlim),length(PlaneYlim),3);  
    for ee = 1:length(PlaneXlim)                                           %Finds location of points on the plane 
        for dd = 1:length(PlaneYlim)
            tx = PlaneXlim(ee);
            ty = PlaneYlim(dd);
            tz = Plane(4)- (Plane(1)*tx + Plane(2)*ty)/Plane(3);
            store(ee,dd,:) = [tx,ty,tz];
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
            plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld,'HandleVisibility','off')
            if colorOld(1) == 'b'
                plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'red','LineWidth',2);
                
            end
            Intersection(ii) = cc;
            ii = ii +1;
            scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc),20, 'black', 'filled','HandleVisibility','off')
            ccOld = cc;
            colorOld = color;
        elseif color(1) ~= colorOld(1) && flag == 0
            %Sets the exit point of the device 
            flag = 1;
            ccOld = cc;
            colorOld = ':black';
            Intersection(ii) = cc;
            ii = ii +1;
        end
        %plot3(Proj(1,cc-1:cc), Proj(2,cc-1:cc), Proj(3,cc-1:cc), color)
    end
    plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld,'LineWidth',2,'HandleVisibility','off')
    if colorOld(1) == 'b'
        plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'red','LineWidth',2);
    end
    %scatter3(Proj(1,Intersection(1)), Proj(2,Intersection(1)), Proj(3,Intersection(1)),20,  'green','filled','HandleVisibility','off')
    %scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc),20, 'green','HandleVisibility','off')
    
    axis equal vis3d
    lim = .05;
    xlim([-lim lim])
    ylim([-lim lim])
    zlim([.075 .175])
    f = gcf();
    set(f,'units','pixels', 'OuterPosition',[(1920-1000)/2 (1080-1000)/2 1000 1000])
    %set(f, 'Render','painters');
    set(f,'GraphicsSmoothing','on');
    hold off
    

    Inter = Intersection(1:ii);
    
elseif mode == 1 
    %%
    set(0, 'CurrentFigure', F)
    color = 'black';
    colorOld = 'black';
    ccOld = 1;                                              
    flag = 0;
    hold on 
    PlaneSize = size(plane);
    PositionSize = size(position);
    Intersection = zeros(1000,1);
    ii = 1;
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
            plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld,'HandleVisibility','off')
            if colorOld(1) == 'b'
                plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'red','LineWidth',2);
                
            end
            Intersection(ii) = cc;
            ii = ii +1;
            scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc),20, 'black', 'filled','HandleVisibility','off')
            ccOld = cc;
            colorOld = color;
        elseif color(1) ~= colorOld(1) && flag == 0
            %Sets the exit point of the device 
            flag = 1;
            ccOld = cc;
            colorOld = ':black';
            Intersection(ii) = cc;
            ii = ii +1;
        end
        %plot3(Proj(1,cc-1:cc), Proj(2,cc-1:cc), Proj(3,cc-1:cc), color)
    end
    plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld,'LineWidth',2,'HandleVisibility','off')
    if colorOld(1) == 'b'
        plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'red','LineWidth',2);
    end
    %scatter3(Proj(1,Intersection(1)), Proj(2,Intersection(1)), Proj(3,Intersection(1)),20,  'green','filled','HandleVisibility','off')
    %scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc),20, 'green','HandleVisibility','off')
    
    axis equal vis3d
    lim = .05;
    xlim([-lim lim])
    ylim([-lim lim])
    zlim([.075 .175])
    f = gcf();
    set(f,'units','pixels', 'OuterPosition',[(1920-1000)/2 (1080-1000)/2 1000 1000])
    hold off
    

    Inter = Intersection(1:ii);
elseif mode == 2
    Intersection = zeros(1000,1);
    ii = 1;
    color = 'black';
    colorOld = 'black';
    ccOld = 1;
    flag = 0;
    
    
    szPl = size(plane);
    if szPl(2) == 4 || szPl(1) == 4
        Po = [plane(1,1) plane(1,2) plane(1,3)];
        R = plane(1,4);
    else
        Po = [plane(1,2) plane(1,3) plane(1,4)];
        R = plane(1,5);
    end
    
    szP = size(position);
    if szP(2) == 3
        Pss = position;
    elseif szP(2) == 4
        Pss = [position(:,2), position(:,3), position(:,4)];
    end
    Ps = Decimate(Pss,40);
%     if isa(position,'Simulink.SimulationData.Signal')
%         Pss = position.Values.Data;
%         Ps = Decimate(Pss,40);
%     else
%         Pss = [position(:,2), position(:,3), position(:,4)];
%         Ps = Decimate(Pss,40);
%     end
%     
    orgin = scatter3(0,0,0);
    hold on
    
    [x,y,z] = sphere;
    s = surf(x*R + Po(1), y*R + Po(2), z*R + Po(3));
    s.EdgeColor = 'none';
    s.FaceColor = [0 0 0];
    s.FaceAlpha = .5;
    
    s = size(Ps);
    Proj = zeros(4,s(1));
    for cc = 1:max(s)
        Proj(1:3,cc) = (Ps(cc,1:3) - Po) / norm(Ps(cc,1:3) - Po) * R + Po;
        Proj(4,cc) = norm(Ps(cc,1:3) - Po);
        if Proj(4,cc) < R %If the position is below the plane - projection is black solid
            color = 'black';
        elseif Proj(4,cc)  > R %If the device is not in contact with the plane the line is dashed 
            color = ':black';
        end
        
        %Plotting takes place when the device enters or exits the wall (ie.
        %when the color of the line changes. The first part of the line is
        %not plotted (assumed to be inside of the surface). 
        
        if color(1) ~= colorOld(1) && flag == 1 
            plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld,'HandleVisibility','off')
            if colorOld(1) == 'b'
                plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'red','LineWidth',2);
            end
            Intersection(ii) = cc;
            ii = ii +1;
            scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc),20, 'black', 'filled','HandleVisibility','off')
            ccOld = cc;
            colorOld = color;
        elseif color(1) ~= colorOld(1) && flag == 0
            %Sets the exit point of the device 
            flag = 1;
            ccOld = cc;
            colorOld = ':black';
            Intersection(ii) = cc;
            ii = ii +1;
        end
    end
    plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld,'LineWidth',2,'HandleVisibility','off')
    if colorOld(1) == 'b'
        plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'red','LineWidth',2);
    end
    
    set(orgin,'Visible','off');
    axis equal vis3d;
    f = gcf();
    set(f,'units','pixels', 'OuterPosition',[(1920-1000)/2 (1080-1000)/2 1000 1000]);
    hold off;
elseif mode == 3
    Intersection = zeros(1000,1);
    ii = 1;
    color = 'black';
    colorOld = 'black';
    ccOld = 1;
    flag = 0;
    
 szPl = size(plane);
    if szPl(2) == 4 || szPl(1) == 4
        Po = [plane(1,1) plane(1,2) plane(1,3)];
        R = plane(1,4);
    else
        Po = [plane(1,2) plane(1,3) plane(1,4)];
        R = plane(1,5);
    end
    
    szP = size(position);
    if szP(2) == 3
        Pss = position;
    elseif szP(2) == 4
        Pss = [position(:,2), position(:,3), position(:,4)];
    end
    Ps = Decimate(Pss,40);
    
%     
%     if isa(position,'Simulink.SimulationData.Signal')
%         Pss = position.Values.Data;
%         Ps = Decimate(Pss,40);
%     else
%         Pss = [position(:,2), position(:,3), position(:,4)];
%         Ps = Decimate(Pss,40);
%     end
%     
    orgin = scatter3(0,0,0);
    hold on
    
    [x,y,z] = sphere;
    s = surf(x*R + Po(1), y*R + Po(2), z*R + Po(3));
    s.EdgeColor = 'none';
    s.FaceColor = [0 0 0];
    s.FaceAlpha = .5;
    
    s = size(Ps);
    Proj = zeros(4,s(1));
    for cc = 1:max(s)
        Proj(1:3,cc) = (Ps(cc,1:3) - Po) / norm(Ps(cc,1:3) - Po) * R + Po;
        Proj(4,cc) = norm(Ps(cc,1:3) - Po);
        if Proj(4,cc) > R %If the position is below the plane - projection is black solid
            color = 'black';
        elseif Proj(4,cc)  < R %If the device is not in contact with the plane the line is dashed 
            color = ':black';
        end
        
        %Plotting takes place when the device enters or exits the wall (ie.
        %when the color of the line changes. The first part of the line is
        %not plotted (assumed to be inside of the surface). 
        
        if color(1) ~= colorOld(1) && flag == 1 
            plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld,'HandleVisibility','off')
            if colorOld(1) == 'b'
                plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'red','LineWidth',2);
            end
            Intersection(ii) = cc;
            ii = ii +1;
            scatter3(Proj(1,cc), Proj(2,cc), Proj(3,cc),20, 'black', 'filled','HandleVisibility','off')
            ccOld = cc;
            colorOld = color;
        elseif color(1) ~= colorOld(1) && flag == 0
            %Sets the exit point of the device 
            flag = 1;
            ccOld = cc;
            colorOld = ':black';
            Intersection(ii) = cc;
            ii = ii +1;
        end
    end
    plot3(Proj(1,ccOld:cc), Proj(2,ccOld:cc), Proj(3,ccOld:cc), colorOld,'LineWidth',2,'HandleVisibility','off')
    if colorOld(1) == 'b'
        plot3(Ps(ccOld:cc,1),Ps(ccOld:cc,2),Ps(ccOld:cc,3),'red','LineWidth',2);
    end
    
    set(orgin,'Visible','off');
    axis equal vis3d;
    f = gcf();
    set(f,'units','pixels', 'OuterPosition',[(1920-1000)/2 (1080-1000)/2 1000 1000]);
    hold off;

end

figure(gcf);


