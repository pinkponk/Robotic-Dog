function [ x,y,z ] = Trans2cart( v1,v2,v3 )

Y1 = 80;    %värden tagna på måfå i mm
Z1 = 20;
Z2 = 150;
Z3 = 67+20;

r = sqrt(Z2^2+Z3^2-2*Z2*Z3*sind(-v3));

Xa = Z2*sind(v2)+Z3*sind(v2+v3);
x = Xa;

Ya = Y1*cosd(v1)+Z1*sind(v1);
Yb = r*sind(v1)*cosd(v2);
y = Ya+Yb;

Za = Y1*sind(v1)+Z1*cosd(v1);
Zb = r*cosd(v1)*cosd(v2);
z = Za + Zb;
end

