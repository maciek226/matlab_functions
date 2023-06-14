% Copyright Maciej Lacki 2019
% All rights reserved 

function [] = PlotDevice(Q1, Q2, Q3, Pos)
%Plots the Novint Falcon 
A = 60e-3; B = 102.5e-3; C = 14.43e-3; D = 11.25e-3; e = D; F = 25e-3; G = 27.9e-3; R = 36.6e-3; S = 23.09e-3;
phi = [0 2*pi/3 4*pi/3 ];


U = Pos(1); V = Pos(2); W = Pos(3);

R = sqrt(R^2 + S^2);
U1 = R * cos(phi);
V1 = R * sin(phi);
W1 = [0 0 0];


scatter3(U1, V1, W1, 'black', 'filled')
hold on

U2 = U1 + cos(phi) .* transpose((A*cos(Q1)));
V2 = V1 + sin(phi) .* transpose((A*cos(Q1)));
W2 = W1 + transpose(A*sin(Q1));
%scatter3(U2,V2,W2, 'black')

U3 = U2 + cos(phi) .* transpose(e*cos(Q2));
V3 = V2 + sin(phi) .* transpose(e*cos(Q2));
W3 = W2 + transpose(e*sin(Q2));
%scatter3(U3,V3,W3, 'black')

U4 = U3 + cos(phi) .* transpose(B*sin(Q3) .* cos(Q2)) - sin(phi) .* transpose(B*cos(Q3));
V4 = V3 + sin(phi) .* transpose(B*sin(Q3) .* cos(Q2)) + cos(phi) .* transpose(B*cos(Q3));
W4 = W3 + transpose(B*sin(Q3) .* sin(Q2));
%scatter3(U4,V4,W4, 'black')

U5 = U4 + cos(phi) .* transpose(D* cos(Q2));
V5 = V4 + sin(phi) .* transpose(D* cos(Q2));
W5 = W4 + transpose(D*sin(Q2));

scatter3(U5,V5,W5, 'black', 'filled')


xlim([-.12,.12])
ylim([-.12,.12])
zlim([-.01,.23])

plot3([U1(1),U2(1),U3(1),U4(1),U5(1)],[V1(1),V2(1),V3(1),V4(1),V5(1)],[W1(1),W2(1),W3(1),W4(1),W5(1)], 'black','LineWidth',3)
plot3([U1(2),U2(2),U3(2),U4(2),U5(2)],[V1(2),V2(2),V3(2),V4(2),V5(2)],[W1(2),W2(2),W3(2),W4(2),W5(2)], 'black','LineWidth',3)
plot3([U1(3),U2(3),U3(3),U4(3),U5(3)],[V1(3),V2(3),V3(3),V4(3),V5(3)],[W1(3),W2(3),W3(3),W4(3),W5(3)], 'black','LineWidth',3)

scatter3(U,V,W,'red','filled')

% text(0, -.06, 0, num2str(sign(AngVel(3))))
% text(0.015, -.045, 0, 'AV 3')
% text(.01, .01, 0, num2str(sign(AngVel(1))))
% text(0.02,.03 , 0, 'AV 1')
% text(-.065, -.04, 0, num2str(sign(AngVel(2))))
% text(-0.05,-.015 , 0, 'AV 2')

end