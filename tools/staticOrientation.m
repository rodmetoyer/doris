% staticOrientation.m

clearvars; close all; clc;

reldens = 1.0:-0.01:0.95;
bodyLength = 18.0;
%bodyLength = 0.5;
%bodyRadius = 0.05*bodyLength;
rotorRadius = bodyLength;
%a1 = 0.030674846625767; 
a1 = 0.0051; 
s1 = a1*bodyLength;
bodyPrcnt = 0:0.001:0.1;
s3 = bodyPrcnt*bodyLength;
axisScaler = 1.2;
plotcolor = 'none';
%lgnclr = 'w';%[0.8510 0.8510 0.8510];
lgnclr = [0.8510 0.8510 0.8510];
saveplots = true;
savenameapp = 'util';
%figw = 400;
%figh = 300;
figw = 600;
figh = 400;

for i=1:1:length(reldens)
    for j=1:1:length(s3)
        theta(i,j) = atan2d(2*reldens(i)*s1,-2*reldens(i)*s3(j)+(1-reldens(i))*bodyLength);
    end
end

tfig = figure('Position',[800 300 figw figh],'color',plotcolor);
tax = axes('Parent',tfig);
hold(tax,'on');
plot(tax,s3/bodyLength,theta(1,:)-90,'DisplayName',['\sigma = ' num2str(reldens(1),'%3.2f')],'LineWidth',2.0);
for i=2:1:length(reldens)-1
    plot(tax,s3/bodyLength,theta(i,:)-90,'DisplayName',['\sigma = ' num2str(reldens(i),'%3.2f')]);
end
plot(tax,s3/bodyLength,theta(end,:)-90,'DisplayName',['\sigma = ' num2str(reldens(end),'%3.2f')],'LineWidth',2.0);
h = tax.Children;
h(round(length(reldens)/2)).LineStyle = '--';
%xline(tax,0.05);
%hleg = legend([h(end) h(round(length(reldens)/2)) h(1)],'Location','Best','Color',lgnclr);
hleg = legend('Location','Best','Color',lgnclr);
% hleg.Title.String = ['Relative Density (Incr. 0.01)'];
hleg.Title.String = ['Relative Density'];
grid(tax,'on');
%xlabel('a_3');
xlabel('Normalized Center Mass Axial Location (a_3)');
ylabel('Pitch Angle (deg)');
%text(tax,0.022,110,['a_1 = ' num2str(a1,2)],'FontSize',12);
%title(['a_1 = ' num2str(a1,2)]);
tax.XLim = [0 0.1];
tax.YLim = [-100 100];
tax.Color = plotcolor;
tax.FontSize = 12;
if saveplots
export_fig(tfig,['products\images\stillHydrostaticPlot' num2str(a1,2) savenameapp '.png'],'-transparent','-m3');
end

% plot what this looks like
s3indx = round(length(s3)/2);
reldensindx = round(length(reldens)/2);
s3toplot = s3(s3indx);
theta2plot = theta(reldensindx,s3indx);

hfig = figure('Position',[1200 400 320 240],'color',plotcolor);
A = [0;0];
p = [0; -bodyLength/2];
q = [0; bodyLength/2];

cs = [s1; s3toplot];
% start
plot([p(1) q(1)],[p(2) q(2)],'-','Linewidth',2.0,'Color',[0.75 0.75 1]);
hold on
plot([p(1)-rotorRadius p(1)+rotorRadius],[p(2) p(2)],'-','Linewidth',2.0,'Color',[0.75 0.75 0.75]);
plot([q(1)-rotorRadius q(1)+rotorRadius],[q(2) q(2)],'-','Linewidth',2.0,'Color',[0.75 0.75 0.75]);
plot(cs(1),cs(2),'o','MarkerSize',6,'MarkerFaceColor',[0.75 0.75 0.75],'MarkerEdgeColor',[0.75 0.75 0.75]);
plot(p(1),p(2),'o','MarkerFaceColor',[0.75 1 0.75],'MarkerEdgeColor',[0.75 1 0.75]);

% equilibrium
peq = [-bodyLength/2*sind(theta2plot); -bodyLength/2*cosd(theta2plot)];
qeq = [bodyLength/2*sind(theta2plot); bodyLength/2*cosd(theta2plot)];
plot([peq(1) qeq(1)],[peq(2) qeq(2)],'-b','Linewidth',2.5);
plot([peq(1)-rotorRadius*cosd(theta2plot) peq(1)+rotorRadius*cosd(theta2plot)],[peq(2)+rotorRadius*sind(theta2plot) peq(2)-rotorRadius*sind(theta2plot)],'-k','Linewidth',2.5);
plot([qeq(1)-rotorRadius*cosd(theta2plot) qeq(1)+rotorRadius*cosd(theta2plot)],[qeq(2)+rotorRadius*sind(theta2plot) qeq(2)-rotorRadius*sind(theta2plot)],'-k','Linewidth',2.5);
plot(cs(1)*cosd(theta2plot)+cs(2)*sind(theta2plot),-cs(1)*sind(theta2plot)+cs(2)*cosd(theta2plot),'o','MarkerSize',6,'MarkerFaceColor','k','MarkerEdgeColor','k');
plot(peq(1),peq(2),'o','MarkerSize',6,'MarkerFaceColor','g','MarkerEdgeColor','g');
plot(A(1),A(2),'or','MarkerSize',6,'MarkerFaceColor',[1 0 0]);
axis equal
axis([-axisScaler*bodyLength axisScaler*bodyLength -axisScaler*bodyLength axisScaler*bodyLength])
hfig.Children.XTick = [];
hfig.Children.YTick = [];
hfig.Children.Visible = 'off';
%title(['Equilibrium position when \sigma = ' num2str(reldens(reldensindx),'%3.2f') ' and CM_a = ' num2str(s3toplot/bodyLength,'%2.1f') '*L_B']);
hfig.Children.Color = plotcolor;
if saveplots
export_fig(hfig,['products\images\stillHydrostaticIllust' num2str(a1,2) '.png'],'-transparent','-m3');
end