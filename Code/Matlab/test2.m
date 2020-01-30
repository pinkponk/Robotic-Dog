%% TEST2
close all
i = 0;
%figure('units','normalized','outerposition',[0 0 1 1])

xOld = -101;
yOld = 60;
zOld = -200-(xOld/20)^2;
[ v1,v2,v3 ] = inverse_kinematics( xOld,yOld,zOld);
v1Old = v1;
v2Old = v2;
v3Old = v3;
for x =-100:100
    y = 60;
    z = -200-(xOld/20)^2;
    i=i+1;
    

    %gett curr pos xOld, yOld, zOld
    
    dx = (x-xOld);
    dy = (y-yOld);
    dz = (z-zOld);
    r = sqrt(dx^2+dy^2+dz^2);
%     xNorm = xOld+dx/r;
%     yNorm = yOld+dy/r;
%     zNorm = zOld+dz/r;
    
    [ v1,v2,v3 ] = inverse_kinematics(x,y,z);
    [ v1Old,v2Old,v3Old ] = inverse_kinematics(xOld,yOld,zOld);
%     [ v1,v2,v3 ] = inverse_kinematics(xNorm,yNorm,zNorm);
    
    sdv1 = (v1-v1Old)*255
    sdv2 = (v2-v2Old)*255
    sdv3 = (v3-v3Old)*255
    
%     dv1_dz = (Ly*z)/((y^2+z^2)^(3/2)*sqrt(1-Ly^2/(y^2+z^2)))-(y*z)/((1+y^2/abs(z)^2)*abs(z)^3);
%     dv1_dy = (Ly*y)/((y^2+z^2)^(3/2)*sqrt(1-Ly^2/(y^2+z^2)))+1/((1+y^2/abs(z)^2)*abs(z));
%     disp(dv1_dz+dv1_dy)
    
    v1Old = v1;
    v2Old = v2;
    v3Old = v3;
    
    xOld = x;
    yOld = y;
    zOld = z;

  
    
    [ X,Y,Z ] = Trans2cartAndPlot( v1,v2,v3 );
    path(i,1:3) = [ X,Y,Z ];
    view(-100-i/2,18)
    hold on
    plot3(path(:,1),path(:,2),path(:,3))
    hold off
    frame = getframe;
end