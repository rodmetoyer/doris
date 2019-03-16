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
        plot([0, L(1)],[0, L(3)],'r','LineWidth',2.0);
        hold on
        plot([0, D(1)],[0, D(3)],'b','LineWidth',2.0);
        hold off
        axis equal
        title(['Lift and Drag | AoA = ' num2str(aoa)]);
        legend('Lift','Drag');
    end

    % Now try slightly negative aoa
    aoa = -10;
    vr = [vrmag*cosd(aoa),0,vrmag*sind(aoa)];
    [L,D,M] = bs.computeLoads(vr,f);
    if makeplots
        figure
        plot([0, L(1)],[0, L(3)],'r','LineWidth',2.0);
        hold on
        plot([0, D(1)],[0, D(3)],'b','LineWidth',2.0);
        hold off
        axis equal
        title(['Lift and Drag | AoA = ' num2str(aoa)]);
        legend('Lift','Drag');
    end

    % Try a bunch
    if makeplots
    hfig = figure('Color','white','Units','inches','Position',[1,1,12,12]);
    axis([-4 4 -4 4]);
    axis equal
    numfigs = 36;
    columns = 6;
        for i=1:1:numfigs
            aoa = (i-numfigs/2)*10;
            vr = [vrmag*cosd(aoa),0,vrmag*sind(aoa)];
            [L,D,~] = bs.computeLoads(vr,f); % 3rd return is moment. As of 16MAR2019 it is constant. todo(rodney) include moment when functionality is there
                %figure
                subplot(numfigs/columns,columns,i);
                plot([0, L(1)],[0, L(3)],'r','LineWidth',2.0);
                axis([-4 4 -4 4]);
                axis equal
                hold on
                plot([0, D(1)],[0, D(3)],'b','LineWidth',2.0);
                hold off
                
                axis equal
                axis([-4 4 -4 4]);
                title(['AoA = ' num2str(aoa)]);
                %legend('Lift','Drag','Location','Best');        
        end
        suptitle(['\fontsize{16} \color{red}Lift',' \color{black}and ', '\fontsize{16} \color{blue}Drag',' \color{black}Vectors at Select Angles of Attack (AoA)']);
    saveas(hfig,'..\figures\bladesectionsLarge.png');
    end
end