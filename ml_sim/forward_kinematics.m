function T = forward_kinematics(theta, T6tcp)
%FORWARD_KINEMATICS FK theo bang DH CR10A, co ho tro TCP tuy chon.
%   T = forward_kinematics(theta) tra T06 toi flange (vi tri mm).
%   T = forward_kinematics(theta,T6tcp) nhan them bien doi TCP trong frame 6.
%   theta = [q1 ... q6] (radian). T6tcp vi tri tinh bang mm.
%
%   Su dung quy uoc DH chuan:
%   A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
%   Cac offset goc trong bang duoc ap dung ben trong ham.

    validateattributes(theta, {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'theta');
    theta = double(theta(:).');

    % Moi hang: [a_i (mm), alpha_i (rad), d_i (mm), theta offset (rad)]
    dh = [ ...
          0, -pi/2,  176.5,       0;
        607,      0,  193.0, -pi/2;
        568,      0, -127.0,       0;
          0,  pi/2,  125.0,  pi/2;
          0, -pi/2,  125.0,       0;
          0,      0,  111.3,       0];

    T = eye(4);
    for i = 1:6
        a = dh(i, 1);
        alpha = dh(i, 2);
        d = dh(i, 3);
        th = theta(i) + dh(i, 4);

        ct = cos(th);
        st = sin(th);
        ca = cos(alpha);
        sa = sin(alpha);

        A = [ct, -st*ca,  st*sa, a*ct; ...
             st,  ct*ca, -ct*sa, a*st; ...
              0,     sa,     ca,    d; ...
              0,      0,      0,    1];

        T = T * A;
    end

    if nargin >= 2 && ~isempty(T6tcp)
        validateattributes(T6tcp, {'numeric'}, ...
            {'real', 'finite', 'size', [4 4]}, mfilename, 'T6tcp');
        T = T * double(T6tcp);
    end
end
