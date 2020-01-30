function [ v1,v2,v3 ] = inverse_kinematics( x,y,z )

Ly = 55;    %värden tagna på måfå i mm
Lz = 45;
L2 = 123;
L3 = 67+20;

%=====================================
%= Angle v1 is positive when tilting arm upwards
%= Angle v2 is positive when bending leg forward
%= Angle v3 is positive when bedning leg bakwards
%=====================================

v1 = (atand(y/(-z))+acosd(Ly/sqrt(z^2+y^2)))-90;

x1 = 0;
y1 = Ly*cosd(v1)+Lz*sind(v1);
z1 = Ly*sind(v1)-Lz*cosd(v1);

r = sqrt((x1-x)^2+(y1-y)^2+(z1-z)^2);
v3 = 180-acosd((L2^2+L3^2-r^2)/(2*L2*L3));

B = -(z-z1)/cosd(v1);
beta = atand(x/B);
v2 = acosd((r^2+L2^2-L3^2)/(2*r*L2))+beta;

end

