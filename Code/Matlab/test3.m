%% TEST2
close all;clear all, clc
figure('units','normalized','outerposition',[0 0 1 1])
xstart = -60;
xend = 60;
steps = 25;
ii = -30:(30--30)/30:30;
x = xstart:-(xstart-xend)/steps:xend;
z =-115-70-0-(x./8-0/6).^2;
%z = -252+x.*0.00001;
y = -30+(x./4000000).^2+8%+ii;


xcur = -110;
zcur = -150;
ycur = 60;
[ v1Old,v2Old,v3Old ] = inverse_kinematics( xcur,ycur,zcur);

k = 1;
for ii = 1:4
    for i = 1:length(x)


        [ v1,v2,v3 ] = inverse_kinematics( x(i),y(i),z(i));
        dv1 = v1-v1Old;
        dv2 = v2-v2Old;
        dv3 = v3-v3Old;

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
        view(-100-50,18)
        hold off
    end
    for i = 1:length(x)



    end

end