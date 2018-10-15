function pass = fluidUnitTest(obj)
% Unit tests for a fluid object
% INPUTS
    % fluid - object (instance of fluid class)
% OUTPUTS
    % pass - boolean (true if all unit tests passed, false otherwise)
    
pass = true;
% Is this a fluid object?
if ~isa(obj,'fluid')
    pass = false;
    return;
end


