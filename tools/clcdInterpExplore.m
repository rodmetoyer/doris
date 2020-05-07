% clcdInterpExplore.m

clear all; close all; clc;

% Need to see what getCLCD function is right
% First make some airfoils
af = airfoil('NACA0015');
fig1 = figure;
plot(af.clcurve(1,:),af.clcurve(2,:),'xr');
hold on
plot(af.cdcurve(1,:),af.cdcurve(2,:),'xb');
hold off
title('NACA0015 CL/CD data');

af2 = airfoil('NACA0009');
aoa = -180:1.5:180;
cl1 = ppval(af.clpp,aoa);
cd1 = ppval(af.cdpp,aoa);
cl2 = ppval(af2.clpp,aoa);
cd2 = ppval(af2.cdpp,aoa);
if true
    figure
    plot(aoa,cl1,'LineWidth',2.0);
    hold on
    plot(aoa,cd1,'LineWidth',2.0);
    plot(aoa,cl2,'LineWidth',2.0);
    plot(aoa,cd2,'LineWidth',2.0);
    hold off
    title(['Force Coefficients for ' af.airfoilName ' airfoil']);
    xlabel('Angle of Attack (deg)'); ylabel(['\color{red}C_L','\color{black} | ','\color{blue}C_D']);
    legend('15 C_L','15 C_D','09 C_L','09 C_D','Location','Best');
end

return

af2 = airfoil('SG6040');
fig2 = figure;
plot(af2.clcurve(1,:),af2.clcurve(2,:),'xr');
hold on
plot(af2.cdcurve(1,:),af2.cdcurve(2,:),'xb');
hold off
title('SG6040 CL/CD data');

% Let's look at some interpolation methods
%aoa = [-180:1:-10 -9.75:0.25:10 11:1:180];
aoa = -181:0.25:181;
% Linear interp
tic
cl1 = interp1(af.clcurve(1,:),af.clcurve(2,:),aoa);
cd1 = interp1(af.cdcurve(1,:),af.cdcurve(2,:),aoa);
tinterp.af1 = toc;
hold(fig1.CurrentAxes,'on');
plot(fig1.CurrentAxes,aoa,cl1,'r',aoa,cd1,'b');
hold(fig1.CurrentAxes,'off');
tic
cl1 = interp1(af2.clcurve(1,:),af2.clcurve(2,:),aoa);
cd1 = interp1(af2.cdcurve(1,:),af2.cdcurve(2,:),aoa);
tinterp.af2 = toc;
hold(fig2.CurrentAxes,'on');
plot(fig2.CurrentAxes,aoa,cl1,'r',aoa,cd1,'b');
hold(fig2.CurrentAxes,'off');
% Spline
tic
cl1 = spline(af.clcurve(1,:),af.clcurve(2,:),aoa);
cd1 = spline(af.cdcurve(1,:),af.cdcurve(2,:),aoa);
tspline.af1 = toc;
hold(fig1.CurrentAxes,'on');
plot(fig1.CurrentAxes,aoa,cl1,':r',aoa,cd1,':b');
hold(fig1.CurrentAxes,'off');
tic
cl1 = spline(af2.clcurve(1,:),af2.clcurve(2,:),aoa);
cd1 = spline(af2.cdcurve(1,:),af2.cdcurve(2,:),aoa);
tspline.af2 = toc;
hold(fig2.CurrentAxes,'on');
plot(fig2.CurrentAxes,aoa,cl1,':r',aoa,cd1,':b');
hold(fig2.CurrentAxes,'off');
% Note that spline has trouble on the ends and linear interpolation is
% obviously faster

% Dheepak's solution was to fit a piecewise function, but it looks like he
% abandoned that idea. Let's try it for the SG6040_24
% polyord = 3;
% [pd1,Sd1] = polyfit(af2.cdcurve(1,1:119),af2.cdcurve(2,1:119),polyord);
% [pd2,Sd2] = polyfit(af2.cdcurve(1,120:350),af2.cdcurve(2,120:350),polyord+2);
% [pd3,Sd3] = polyfit(af2.cdcurve(1,351:471),af2.cdcurve(2,351:471),polyord+2);
% [pl1,Sl1] = polyfit(af2.clcurve(1,1:119),af2.clcurve(2,1:119),polyord);
% [pl2,Sl2] = polyfit(af2.clcurve(1,120:350),af2.clcurve(2,120:350),polyord);
% [pl3,Sl3] = polyfit(af2.clcurve(1,351:471),af2.clcurve(2,351:471),polyord);
% CL = NaN(size(aoa));
% CD = CL;
% for i=1:1:length(aoa)
%     if(aoa(i) < -180 || aoa(i) > 180)
%         x = mod(aoa(i)+180,180);
%     else
%         x = aoa(i);
%     end
%     x = round(x);
% %     if(x>=-180 && x<=-62)
% %         CL(i)= pl1(1)*(x^3) + pl1(2)*(x^2) + pl1(3)*x +pl1(4);
% %         CD(i)= pd1(1)*(x^3) + pd1(2)*(x^2) + pd1(3)*x +pd1(4);
% %     elseif(x>-62 && x<60)
% %         CL(i)= pl2(1)*(x^3) + pl2(2)*(x^2) + pl2(3)*x +pl2(4);
% %         CD(i)= pd2(1)*(x^5) + pd2(2)*(x^4) + pd2(3)*(x^3) + pd2(4)*(x^2) + pd2(5)*x +pd2(5);
% %     elseif(x>=60 && x<=180)
% %         %x = mu3(1) + x;
% %         CL(i)= pl3(1)*(x^3) + pl3(2)*(x^2) + pl3(3)*x +pl3(4);
% %         CD(i)= pd3(1)*(x^5) + pd3(2)*(x^4) + pd3(3)*(x^3) + pd3(4)*(x^2) + pd3(5)*x +pd3(5)+3.2;
% %     else
% %         error('Angle of attack must be between -180 and 180, inclusive');
% %     end
% end
fig3 = figure;
plot(af2.clcurve(1,:),af2.clcurve(2,:),'xr');
hold on
plot(af2.cdcurve(1,:),af2.cdcurve(2,:),'xb');
% plot(aoa,CL,':r',aoa,CD,':b');
hold off
title('SG6040 CL/CD data and Poly Fit');
