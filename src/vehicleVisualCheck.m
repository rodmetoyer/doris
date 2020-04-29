function hfig = vehicleVisualCheck(f,v)
error('vehicleVisualCheck is deprecated. Use sim.showme.');
% Creates a plot to visually verify that the vehicle you constructed is the
% vehicle you were trying to construct

% Get plot arrays from subfunction
[rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,~,~,~,~] = getPlotArrays(v,f);
x = 300; y = 100; w = x+600; h = y+600;
hfig = figure('position',[x y w h]);
scale = 1;
y = linspace(-v.rotors(1).blades(1).length,v.rotors(1).blades(1).length,10);
z = v.position(3)+y;
[Y,Z] = meshgrid(y, z);
X = zeros(size(Y));
U = ones(size(X))*f.velocity(1); V = ones(size(Y))*f.velocity(2); W = ones(size(Z))*f.velocity(3);
%quiver3(0,0,0,f.velocity(1),f.velocity(2),f.velocity(3),scale,'b');
quiver3(X,Y,Z,U,V,W,scale,'b');
hold on
vline = [v.position+transpose(v.A_C_O)*v.rotorLocs(:,1),v.position+transpose(v.A_C_O)*v.rotorLocs(:,2)];
plot3(vline(1,:),vline(2,:),vline(3,:),':','LineWidth',3,'color',[0.6350 0.0780 0.1840]);
quiver3(rp1o_O(1)+rap1_O(1,:,:),rp1o_O(2)+rap1_O(2,:,:),rp1o_O(3)+rap1_O(3,:,:),Urel1_O(1,:,:),Urel1_O(2,:,:),Urel1_O(3,:,:),scale,'c')
quiver3(rp2o_O(1)+rap2_O(1,:,:),rp2o_O(2)+rap2_O(2,:,:),rp2o_O(3)+rap2_O(3,:,:),Urel2_O(1,:,:),Urel2_O(2,:,:),Urel2_O(3,:,:),scale,'g')
axis equal
xlabel('x'); ylabel('y'); zlabel('z');
title(['Velocity Vectors | p_3 = ' num2str(v.rotors(1).angvel(3)) ' and q_3 = ' num2str(v.rotors(2).angvel(3))]);
legend({'Freestream Velocity','Vehicle Body','Relative Velocity Rotor 1','Relative Velocity Rotor 2'},'Location','bestoutside','color','none','Orientation','horizontal');
view(-130,20)
hold off
end

% subfunction
function [rp1o_O,rp2o_O,rap1_O,rap2_O,Urel1_O,Urel2_O,L1_O,D1_O,L2_O,D2_O] = getPlotArrays(v,f)
    Urel1_P1 = v.rotors(1).computeHydroLoads(f);
    Urel2_P2 = v.rotors(2).computeHydroLoads(f);
    temp = size(v.rotors(1).sectPos);
    O_C_A = transpose(v.A_C_O);
    O_C_P1 = O_C_A*transpose(v.rotors(1).P_C_A);
    O_C_P2 = O_C_A*transpose(v.rotors(2).P_C_A);
    for i=1:1:temp(2)
        for j=1:1:temp(3)
            rap1_O(:,i,j) = O_C_P1*v.rotors(1).sectPos(:,i,j);
            rap2_O(:,i,j) = O_C_P2*v.rotors(2).sectPos(:,i,j);
            Urel1_O(:,i,j) = O_C_P1*Urel1_P1(:,i,j);
            Urel2_O(:,i,j) = O_C_P2*Urel2_P2(:,i,j);
            L1_O(:,i,j) = O_C_P1*v.rotors(1).sectLift(:,i,j);
            D1_O(:,i,j) = O_C_P1*v.rotors(1).sectDrag(:,i,j);
            L2_O(:,i,j) = O_C_P2*v.rotors(2).sectLift(:,i,j);
            D2_O(:,i,j) = O_C_P2*v.rotors(2).sectDrag(:,i,j);
        end
    end
    % Show the forces and/or velocity vectors as quivers applied at the section
    % locations. For that we need rp1o_O and rp2o_O
    rco_O = v.position;
    rp1c_O = O_C_A*v.rotorLocs(:,1);
    rp2c_O = O_C_A*v.rotorLocs(:,2);
    rp1o_O = rco_O + rp1c_O;
    rp2o_O = rco_O + rp2c_O;
end