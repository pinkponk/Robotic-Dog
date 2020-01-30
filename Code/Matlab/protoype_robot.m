clear all;clc;close all;
%Cord sys:
%X = forward
%Y = sideways + -> left when looking forward
%Z = upwards


%% XZ RED
subplot(4,4,1)
Xvalues = [0, 0.5, 1]';
Yvalues = [0, 0.5, 0]';

plot(Xvalues,Yvalues,'*')
axis equal
hold on

P11 = polyfit(Xvalues,Yvalues,2);

X = 0:0.001:1;
plot(X,polyval(P11,X),'r');
title('Gait curve XZ');xlabel('X');ylabel('Z')
%% XY GREEN
subplot(4,4,2)
Yvalues = [0, 2, 0]';

plot(Xvalues,Yvalues,'*')
axis equal
hold on

P12 = polyfit(Xvalues,Yvalues,2);

plot(X,polyval(P12,X),'g');
title('Gait curve XY');xlabel('X');ylabel('Y')
%%
subplot(4,4,3)
plot3(X,polyval(P12,X),polyval(P11,X));
title('Gait curve XYZ');xlabel('X');ylabel('Y');zlabel('Z');axis equal


%% SPEED
subplot(4,4,4)
Xvalues = [0, 0.5, 1]';
Yvalues = [10, 30, 10]';

plot(Xvalues,Yvalues,'*')

P13 = polyfit(Xvalues,Yvalues,2);

plot(X,polyval(P13,X));
title('Speed');xlabel('time');ylabel('speed');hold on

%%====================================================================================




%%
subplot(4,4,5)
Xvalues = [0, 1]';
Zvalues = [0, 0]';

plot(Xvalues,Zvalues,'*')
axis equal
hold on

P21 = polyfit(Xvalues,Zvalues,1);

X = 0:0.001:1;
plot(X,polyval(P21,X));
title('Gait curve XZ');xlabel('X');ylabel('Z')
%%
subplot(4,4,6)
Xvalues = [0, 1]';
Yvalues = [0, 0]';

plot(Xvalues,Yvalues,'*')
axis equal
hold on

P22 = polyfit(Xvalues,Yvalues,1);

plot(X,polyval(P22,X));
title('Gait curve XY');xlabel('X');ylabel('Y')
%%
subplot(4,4,7)
plot3(X,polyval(P22,X),polyval(P21,X));
title('Gait curve XYZ');xlabel('X');ylabel('Y');zlabel('Z');axis equal

%%
subplot(4,4,8)
Xvalues = [0, 1]';
Yvalues = [10, 10]';

plot(Xvalues,Yvalues,'*')

P23 = polyfit(Xvalues,Yvalues,1);

plot(X,polyval(P23,X));
title('Speed');xlabel('time');ylabel('speed');hold on


%%========================================================================================

subplot(4,4,11)
plot3(X,polyval(P12,X),polyval(P11,X));
hold on
plot3(X,polyval(P22,X),polyval(P21,X));
title('Gait curve XYZ');xlabel('X');ylabel('Y');zlabel('Z');axis equal

%%========================================================================================

subplot(4,4,9);hold on
XZder = polyder(P11);
XYder = polyder(P12);
DZ = polyval(XZder,X);
DY = polyval(XYder,X);
DX = 1;

plot(X,DZ,'r')
plot(X,DY,'g')
plot(X,1,'k')
plot(X,DY.*DZ,'m')




%%
subplot(4,4,10);hold on
Dtot = abs(DX)+abs(DY)+abs(DY);

plot(X,polyval(P13,X).*DX,'k');
plot(X,polyval(P13,X).*DY./Dtot,'g');
plot(X,polyval(P13,X).*DZ./Dtot,'r');
legend()
title('Travel Speed');xlabel('X');ylabel('speed');


