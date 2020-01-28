function animate(v,fn)
    % Makes an animation of the data in file fn
    % v is a vehicle object
    % fn is a file name from the data folder
    % todo put this in the simulation class
    A = importdata(['products\data\' fn]);
    moviefile = ['products\videos\' fn(1:end-4) '.avi'];
    % Just assume the structure for both single and dual
    [rows,colmns] = size(A.data);
    if colmns == 13 % Single rotor system
        % parse the data
        y = A.data;
        % make the movie
        disp('Making a movie of the motion');
        f1 = figure;
        f1.Position = [0 0 0.9*1920 0.9*1080];
        f1.Color = [1 1 1];
        for i = 1:1:rows
            cthe = cos(y(i,1)); sthe = sin(y(i,1));
            cgam = cos(y(i,2)); sgam = sin(y(i,2));
            cbet = cos(y(i,3)); sbet = sin(y(i,3));
            B_C_O = [cbet*cthe + sbet*sgam*sthe, cgam*sbet, sbet*cthe*sgam - cbet*sthe;...
                cbet*sgam*sthe - sbet*cthe, cbet*cgam, sbet*sthe + cbet*cthe*sgam;...
                cgam*sthe,-sgam,cgam*cthe];
            O_C_B = transpose(B_C_O);
            r_b1cm_O = O_C_B*v.rotors(1).sectPos(:,:,1);
            r_b2cm_O = O_C_B*v.rotors(1).sectPos(:,:,2);
            r_b3cm_O = O_C_B*v.rotors(1).sectPos(:,:,3);
            figure(f1);
            plot3([r_cmO_O(i,1) r_cmO_O(i,1)+r_b1cm_O(1)],[r_cmO_O(i,2) r_cmO_O(i,2)+r_b1cm_O(2)],[r_cmO_O(i,3) r_cmO_O(i,3)+r_b1cm_O(3)],'r');
            hold on
            plot3([r_cmO_O(i,1) r_cmO_O(i,1)+r_b2cm_O(1)],[r_cmO_O(i,2) r_cmO_O(i,2)+r_b2cm_O(2)],[r_cmO_O(i,3) r_cmO_O(i,3)+r_b2cm_O(3)],'b');
            plot3([r_cmO_O(i,1) r_cmO_O(i,1)+r_b3cm_O(1)],[r_cmO_O(i,2) r_cmO_O(i,2)+r_b3cm_O(2)],[r_cmO_O(i,3) r_cmO_O(i,3)+r_b3cm_O(3)],'k');
            axis equal
            axis([-0.25 0.25 -0.25 0.25 -0.5 0.25]);

            view(-80,15)
            xlabel('x'); ylabel('y');
            %title(['\fontsize{20}U_\infty = ' num2str(0.5/(1+exp(-0.5*(t(i)-10))),2)]);
            %water.rampvelocity(t(i));
            title(['\fontsize{20}RPM = ' num2str(abs(y(i,9)/(2*pi)*60),'%5.2f'), ' U_\infty = ' num2str(water.velocity(1),'%5.2f')]);
            F(i) = getframe(f1);    
            hold off
        end

        % sm = input('Do you want to save the movie file? Y/N [Y]: ', 's');
        % if isempty(sm)
        %     sm = 'Y';
        % end
        % todo make an interface for v props
        % if strcmp(sm,'Y')

        vw = VideoWriter(moviefile);
        vw.FrameRate = round(1/tstep)*speedfactor;
        %v.Quality = 100;% v.Width = 800; w.Height = 450;
        open(vw);
        writeVideo(vw,F); close(vw);
    elseif colmns == 17 % Dual rotor system
    t = A.data(:,1);
    x = A.data(:,2);
    y = A.data(:,3);
    z = A.data(:,4);
    theta = A.data(:,5);
    gamma = A.data(:,6);
    beta = A.data(:,7);
    fi3 = A.data(:,15);
    sy3 = A.data(:,17);    
    % loop over all of the data and animate everything in the O frame
    
    else
        error('Unknown data file format. Must be dual or single rotor');
    end    
end