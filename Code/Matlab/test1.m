clear all;clc; close all



%[ v1,v2,v3 ] = inverse_kinematics( -50,30,-260);
% [ X,Y,Z ] = Trans2cartAndPlot( v1,v2,v3 );
    
%writerObj = VideoWriter('peaks.avi');
%open(writerObj);

i = 0;
figure('units','normalized','outerposition',[0 0 1 1])
for x =-100:100
    i=i+1;
    [ v1,v2,v3 ] = inverse_kinematics( x,60,-200-(x/20)^2);
    [ X,Y,Z ] = Trans2cartAndPlot( v1,v2,v3 );
    path(i,1:3) = [ X,Y,Z ];
    view(-100-i/2,18)
    hold on
    plot3(path(:,1),path(:,2),path(:,3))
    hold off
    frame = getframe;
    %writeVideo(writerObj,frame);
end
%close(writerObj);