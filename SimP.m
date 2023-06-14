% Copyright Maciej Lacki 2019
% All rights reserved 

function [] = SimP(P,Pl,mode)
if isa(P,'Simulink.SimulationData.Signal')
    Ps = P.Values.Data;
elseif isdouble(P)
    sz = size(P);
    if min(sz) > 3 && min(sz) == sz(1)
        Ps = P(2:4,:);
    elseif min(sz) > 3 && min(sz) == sz(2)
        Ps = P(:,2:4);
    else
        Ps = P;
    end
end
if isa(Pl,'Simulink.SimulationData.Signal')
    Pla = Pl.Values.Data;
    sz = size(Pla);

end
Plane = [0 0 .1 .03];
%Plane = [Plane(1,2) Plane(1,3) Plane(1,4) Plane(1,5)];
%Ps = [P(:,2), P(:,3), P(:,4)];
%Ps = P';
orgin = scatter3(0,0,0);

set(orgin,'Visible','off')
hold on 

if mode == 0
    %%
    store = zeros(10,10,3);

    for ee = 1:10
        for dd = 1:10
            tx = -.075+ee*.015;
            ty = -.075+dd*.015;
            %scatter3(tx , ty, -Plane(4) - (Plane(1)*tx + Plane(2)*ty)/Plane(3) , 10, 'black' , 'filled' )
            store(ee,dd,:) = [tx,ty,Plane(4)- (Plane(1)*tx + Plane(2)*ty)/Plane(3)];
        end
    end

    s = surf(store(:,:,1),store(:,:,2),store(:,:,3));
    s.EdgeColor = 'none';
    s.FaceColor = [0 0 0];
    s.FaceAlpha = .5;
elseif mode == 1
    [x,y,z] = sphere;
    s= surf(x*Plane(4)+Plane(1),y*Plane(4)+Plane(2),z*Plane(4)+Plane(3));
    s.EdgeColor = 'none';
    s.FaceColor = [0 0 0];
    s.FaceAlpha = .5;
    
elseif mode == 2
    [x,y,z] = sphere;
    s= surf(x*Plane(4)+Plane(1),y*Plane(4)+Plane(2),z*Plane(4)+Plane(3));
    s.EdgeColor = 'none';
    s.FaceColor = [0 0 0];
    s.FaceAlpha = .5;
    
end

%Ps = [smooth(P(:,1)), smooth(P(:,2)), smooth(P(:,3))];
plot3(Ps(:,1),Ps(:,2),Ps(:,3),'blue','LineWidth',1);
hold off


xlim([-.1 .1])
ylim([-.1 .1])
zlim([0 .2])
axis equal vis3d
f = gcf();
set(f,'units','pixels', 'OuterPosition',[(1920-1000)/2 (1080-1000)/2 1000 1000])
end