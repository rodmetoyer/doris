% statOrientEquil.m
% For looking at the theoretical static equilibrium orientations

clearvars; close all; clc;

imagedir = '..\products\analysis\EFS\';
color = [0.80 0.80 0.80];

a1 = 0.0051063829787234;
a3 = 0.0191489361702128;
sgma = 0.95;
theta = 0:0.01:2*pi;
mom = 0.5*sin(theta)+a3*sin(theta)+a1*cos(theta)-0.5/sgma*sin(theta);

hfig = figure('Color','w');
ax = axes('Parent',hfig,'Color','none');
plot(ax,theta*180/pi-90,mom)
xlabel('Pitch (deg)');
ylabel('Normalized Moment');
patch(ax,[0 0 180 180],[ax.YLim(1) ax.YLim(2) ax.YLim(2) ax.YLim(1)],'g','FaceAlpha',0.2);
%xline(0);
text(ax,60,ax.YLim(2)*0.5,'Nose up region')
%xline(180);
yline(0);
grid(ax,'on');
ax.Color = 'none';
export_fig(hfig,[imagedir 'EFScase58MomentThetaTheory.png'],'-png','-transparent','-m3');

sgma = 1.0;
a1 = 0:0.001:0.5;
a3 = 0:0.001:0.1;
[A1,A3] = meshgrid(a1,a3);
theta = atan2d(2*A1*sgma,(1-sgma)-2*A3*sgma)-90;
hfig2 = figure('Position',[100 100 600 500]);
ax2 = axes('Parent',hfig2,'Color','none');
s1 = surf(ax2,A1,A3,theta,'FaceAlpha',0.5,'EdgeColor','none','DisplayName',num2str(sgma,2),'FaceColor','r');
hold(ax2,'on');
sgma = 0.92;
theta = atan2d(2*A1*sgma,(1-sgma)-2*A3*sgma)-90;
surf(ax2,A1,A3,theta,'FaceAlpha',0.5,'EdgeColor','none','DisplayName',num2str(sgma,2),'FaceColor','b')
sgma = 0.88;
theta = atan2d(2*A1*sgma,(1-sgma)-2*A3*sgma)-90;
surf(ax2,A1,A3,theta,'FaceAlpha',0.5,'EdgeColor','none','DisplayName',num2str(sgma,2),'FaceColor','y')
sgma = 0.84;
theta = atan2d(2*A1*sgma,(1-sgma)-2*A3*sgma)-90;
surf(ax2,A1,A3,theta,'FaceAlpha',0.5,'EdgeColor','none','DisplayName',num2str(sgma,2),'FaceColor','g')
ax2.Color = 'none';
xlabel('Normalized Radial Position');
ylabel('Normalized Axial Position');
zlabel('Pitch (deg)');
view(-46,5)
lgn = legend;
lgn.Title.String = 'Relative Density';
lgn.Color = 'none';%[0.8510 0.8510 0.8510];
export_fig(hfig2,[imagedir 'EquilibriumThetaTheory58.png'],'-png','-transparent','-m3');

% Make a video
hfig2.Color = color;
ax2.Color = color;
lgn.Color = color;
vwtr = VideoWriter([imagedir 'theoryVid.avi']);
vwtr.Quality = 100;
open(vwtr);
for i=1:1:360
    view(-46+i,5);
    frm = getframe(hfig2);
    writeVideo(vwtr,frm);
end
close(vwtr);
vwtr = VideoWriter([imagedir 'theoryVid58.avi']);
