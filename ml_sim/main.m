function [solutions, positionErrors, orientationErrors] = main(Ttarget, T6tcp)
%MAIN Demonstrate CR10A IK/TCP FK round-trip in this folder.
%   With no inputs, makes a reachable TCP target from a known joint vector.
%   Ttarget is the desired TCP pose (translation in mm). T6tcp is the fixed
%   transform from frame 6 to the TCP; if omitted it matches SampleScene.
%   Outputs are IK rows [q1 ... q6] in radians and FK residuals.

    if nargin < 2 || isempty(T6tcp)
        T6tcp = eye(4);
        % SampleScene TCP: local z = +100 mm; local rotation = -90 deg about Y.
        T6tcp(1:3,1:3) = [0, 0, -1; 0, 1, 0; 1, 0, 0];
        T6tcp(3,4) = 100;
    end
    validateattributes(T6tcp, {'numeric'}, ...
        {'real','finite','size',[4 4]}, mfilename, 'T6tcp');

    if nargin < 1 || isempty(Ttarget)
        qExample = deg2rad([-30, 30, 105, 20, -30, 100]);
        Ttarget = forward_kinematics(qExample, T6tcp);
    end
    validateattributes(Ttarget, {'numeric'}, ...
        {'real','finite','size',[4 4]}, mfilename, 'Ttarget');
    Ttarget = double(Ttarget);

    T06 = Ttarget / double(T6tcp);
    p05 = T06(1:3,4) - 111.3 * T06(1:3,3);
    fprintf('Frame-5 point p05 (mm):\n');
    disp(p05.');

    [solutions, positionErrors, orientationErrors] = IK(Ttarget, T6tcp);
    fprintf('Found %d IK branch solution(s), angles in degrees:\n', size(solutions,1));
    if isempty(solutions)
        return;
    end
    disp(rad2deg(solutions));

    fprintf('IK -> TCP FK residuals:\n');
    for i = 1:size(solutions,1)
        fprintf('Solution %02d: position = %.6f mm, orientation = %.6f deg\n', ...
            i, positionErrors(i), rad2deg(orientationErrors(i)));
    end
end
