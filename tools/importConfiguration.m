function cf = importConfiguration(fn)
% imports a configuration from file

% There must be a quicker way to do this. Doesn't need to be bullet proof
% right now.
T = readtable('test.csv');
Tcell = table2cell(T);
% 