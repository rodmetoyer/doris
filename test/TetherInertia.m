% TetherInertia.m
clear all; close all; clc;
n = 1:2:13;
rhotb = [inf logspace(3,-2,6)];
whichToDisp = 1;

hfig = figure;
ax1 = axes('Parent',hfig);
hold on
for k=1:1:length(rhotb)
    exct = 1/3*(1+3/rhotb(k));
    for j=1:1:length(n)
        temp=0;    
        for i=1:1:n(j)
            temp = temp + (i./n(j))^2;
        end
        nr1 = 1/(n(j)+1)*temp + 1/rhotb(k);
        errp1(k,j) = (nr1-exct)/exct*100;

        temp=0;
        for i=1:1:n(j)
            temp = temp + ((i-1)/n(j))^2;
        end
        nr2 = 1/(n(j))*temp + 1/rhotb(k);
        errp2(k,j) = (nr2-exct)/exct*100;
    end
    lw = 1.0;
    lt = ':';
    if k == numel(rhotb)
        lw = 2.0;
        lt = '-'
    end
    plot(ax1,n,errp1(k,:),lt,'DisplayName',['Included \rho=' num2str(rhotb(k))],'LineWidth',lw);
    plot(ax1,n,errp2(k,:),lt,'DisplayName',['Excluded \rho=' num2str(rhotb(k))],'LineWidth',lw);
end
hold off
ax1.XTick = 1:1:n(end);
axis([1 inf -100 50]);
legend('Location','Best','color','w');
title('Percent error in the inertia of a straight tether');
xlabel('Number of links'); ylabel('Percent Error');
str = ['Error with ' num2str(n(end),'%10.0f') ' links'];
disp(str);
str = ['Method 1: ' num2str(errp1(whichToDisp,end),5) '%'];
disp(str);
str = ['Method 2: ' num2str(errp2(whichToDisp,end),5) '%'];
disp(str);
set(ax1,'Color','none');
export_fig(hfig,'output\figs\withEndErrVsLinks.png','-png','-transparent');

hfigs = figure;
axs = axes('Parent',hfigs);
plot(axs,n,errp1(whichToDisp,:),'r',n,errp2(whichToDisp,:),'b');
axis([1 inf -100 50])
legend('Included','Excluded','Location','Best','Color','none');
title('Percent error in the inertia of a straight tether');
xlabel(axs,'Number of links'); ylabel(axs,'Percent Error');
set(axs,'Color','none');
%export_fig(hfigs,'output\figs\noEndErrVsLinks.png','-png','-transparent');

hfig2 = figure('Position',[200 200 900 600]);
ax2 = axes('Parent',hfig2);
semilogx(ax2,rhotb,errp1)
hold on
semilogx(ax2,rhotb,errp2)
hold off
ax2.XDir = 'reverse';
lgd = legend([string(n) string(n)],'Location','Best','color','none');
lgd.Title.String = "Number of Links";
title('Percent error in the inertia of a straight tether with end body');
xlabel('\rho (mass tether/mass body)'); ylabel('Percent Error');
set(ax2,'Color','none');
export_fig(hfig2,'output\figs\withEndErrVsRho.png','-png','-transparent');