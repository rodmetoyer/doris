function passed = bladesectionUnitTest(bs,f,makeplots)

    passed = true;
    % Try different relative velocities and make sure the lift and drag vectors
    % are appropriate
    vrmag = 1.0;
    % All in the x direction
    aoa = 0;
    vr = [vrmag*cosd(aoa),0,vrmag*sind(aoa)];
    [L,D,M] = bs.computeLoads(vr,f);
    % L should be entirely in z direction, D in x direction, nothing in y
    tol = 10^-10;
    if abs(L(1)) > tol || abs(L(2)) > tol
        warning('Incorrect lift vector');
        passed = false;
    end
    if abs(D(3)) > tol || abs(D(2)) > tol
        warning('Incorrect drag vector');
        passed = false;
    end
    if makeplots
        figure
        plot([0, L(1)],[0, L(3)],'r');
        hold on
        plot([0, D(1)],[0, D(3)],'b');
        hold off
        title(['Lift and Drag | AoA = ' num2str(aoa)]);
        legend('Lift','Drag');
    end

    % Now try slightly negative aoa
    aoa = -10;
    vr = [vrmag*cosd(aoa),0,vrmag*sind(aoa)];
    [L,D,M] = bs.computeLoads(vr,f);
    if makeplots
        figure
        plot([0, L(1)],[0, L(3)],'r');
        hold on
        plot([0, D(1)],[0, D(3)],'b');
        hold off
        title(['Lift and Drag | AoA = ' num2str(aoa)]);
        legend('Lift','Drag');
    end

    % Try a bunch
    if makeplots
    figure('Color','white','Units','inches','Position',[1,1,8,3]);
        for i=1:1:10
            aoa = (i-5)*10;
            vr = [vrmag*cosd(aoa),0,vrmag*sind(aoa)];
            [L,D,M] = bs.computeLoads(vr,f);
                %figure
                subplot(2,5,i);
                plot([0, L(1)],[0, L(3)],'r');
                hold on
                plot([0, D(1)],[0, D(3)],'b');
                hold off
                axis equal
                title(['AoA = ' num2str(aoa)]);
                legend('Lift','Drag','Location','Best');        
        end
    saveas(gcf,'figures\bladesections.png');
    end
end