function [ x4,y4,z4 ] = Trans2cartAndPlot( v1,v2,v3 )

Ly = 55;    %värden tagna på måfå i mm
Lz = 45;
L2 = 123;
L3 = 67+20;


disp('==================');
x1 = 0;
y1 = Ly*cosd(v1);
z1 = Ly*sind(v1);
plot3([0 x1],[0 y1],[0 z1],'g','LineWidth',4)
[X, Y, Z] = cylinder2P(4, 20,[0,0,0],[x1,y1,z1]);
surf(X, Y, Z);
clear X Y Z

grid on
hold on

x2 = 0;
y2 = y1+Lz*sind(v1);
z2 = z1-Lz*cosd(v1);
plot3([x1 x2],[y1 y2],[z1 z2],'r','LineWidth',4)
[X, Y, Z] = cylinder2P(4, 20,[x1,y1,z1],[x2,y2,z2]);
surf(X, Y, Z);

x3 = L2*sind(v2);
y3 = y2+L2*cosd(v2)*sind(v1);
z3 = z2-L2*cosd(v2)*cosd(v1);
plot3([x2 x3],[y2 y3],[z2 z3],'m','LineWidth',4)
[X, Y, Z] = cylinder2P(4, 20,[x2,y2,z2],[x3,y3,z3]);
surf(X, Y, Z);

x4 = x3+L3*sind(v2-v3);
y4 = y3+L3*cosd(v2-v3)*sind(v1);
z4 = z3-L3*cosd(v2-v3)*cosd(v1);
plot3([x3 x4],[y3 y4],[z3 z4],'b','LineWidth',4)
[X, Y, Z] = cylinder2P(4, 20,[x3,y3,z3],[x4,y4,z4]);
surf(X, Y, Z);

[X, Y, Z] = cylinder2P(40, 20,[0,-40,0],[-300,-40,0]);
surf(X, Y, Z);

xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal
%axis([-200 200 -200 200 -400 400])
hold off
% r = sqrt(L2^2+L3^2-2*L2*L3*sind(-v3));
% 
% Xa = L2*sind(v2)+L3*sind(v2+v3);
% x = Xa;
% 
% Ya = Ly*cosd(v1)+Lz*sind(v1);
% Yb = r*sind(v1)*cosd(v2);
% y = Ya+Yb;
% 
% Za = Ly*sind(v1)+Lz*cosd(v1);
% Zb = r*cosd(v1)*cosd(v2);
% z = Za + Zb;
% plot3(x,y,z,'x')
end

