function writeToFile(t,y,flnm)
% Writes vehcile simulation data to file
% INPUTS: 
    % t = time vector
    % y = state matrix
    % flnm = name of the data file
    
% Break into states for writing
[~,nStates] = size(y);
dualrtr = false;
if nStates == 16
    disp('Writing dual-rotor system data to file.');
    frmspc = '%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n';
    dualrtr = true;
elseif nStates == 14
    disp('Writing single-rotor system data to file.');
    frmspc = '%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n';
else
    error('The data you want me to write contains a strange number of states.');
end
headers = ["time","x1","x2","x3","theta","gamma","beta","w1","w2","w3","u1","u2","u3","p3","fi3","q3","sy3"];
fid = fopen(flnm,'w');
if ~dualrtr
    headers = headers(1:end-2);
end
dat = [t,y].';
fprintf(fid,frmspc,headers);
frmspc = strrep(frmspc,'s','f');
fprintf(fid,frmspc,dat);
fclose(fid);
fprintf('\nDone\n');