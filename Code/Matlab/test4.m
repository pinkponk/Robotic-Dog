%% TEST2
close all;clear all, clc
figure('units','normalized','outerposition',[0 0 1 1])
Ly = 55;
Lz = 45;
L2 = 123;
L3 = 67+20;

dir = -1;
turnMagni = 0.5;

%x = -(60+turnMagni*( 5*((1-dir)/2)  -20*((1+dir)/2) )):2:(60+turnMagni*( 5*((1+dir)/2)  -20*((1-dir)/2) ));
x = -60:2:60;  %Turning right
%x = -115:5:90;  %Turning left
z = -218-(x./10).^2%+dir*turnMagni*((115-90)/2)/200.*x;      %2.5/40.*x; is to center the height cruve so start and stop are at the same height
%z=-190+x./1000;
%z = -253+x./100000%+dir*turnMagni*((115-90)/2)/200.*x;
y = sqrt(Ly.^2+(sqrt((L2+L3).^2-x.^2)+Lz).^2-z.^2)%+0.05;        %-0.05 is to avoid imaginary numbers

xcur = -110;
zcur = -150;
ycur = 60;
[ v1Old,v2Old,v3Old ] = inverse_kinematics( xcur,ycur,zcur);

k = 1;
for i = 1:length(x)

   
    [ v1,v2,v3 ] = inverse_kinematics( x(i),y(i),z(i))
    dv1 = real(v1)-v1Old;
    dv2 = real(v2)-v2Old;
    dv3 = real(v3)-v3Old;
    
    v1Old = v1Old + dv1*k;
    v2Old = v2Old + dv2*k;
    v3Old = v3Old + dv3*k;
    frame = getframe;

    %hold on    %om steg synliga
    plot3(x(i),y(i),z(i),'o')
    hold on
    path(i,1:3) = [x(i),y(i),z(i) ];
    plot3(path(:,1),path(:,2),path(:,3))
    [ xcur,ycur,zcur ] = Trans2cartAndPlot( v1Old,v2Old,v3Old );
    view(-100-200,18)
    hold off
  
end