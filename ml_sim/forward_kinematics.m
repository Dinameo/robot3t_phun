function T06 = forward_kinematics(theta)
%FORWARD_KINEMATICS Tinh ma tran dong hoc thuan T06 theo bang DH.
%   T06 = forward_kinematics(theta) nhan theta = [q1 ... q6] (radian)
%   va tra ve ma tran bien doi dong nhat 4x4. Don vi vi tri la mm.
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

    T06 = eye(4);
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

        T06 = T06 * A;
    end
end
