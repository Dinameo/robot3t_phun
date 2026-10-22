function [solutions, positionErrors, orientationErrors] = IK(Ttarget, T6tcp)
%IK Analytic DH branches with d5 position/orientation coupling for CR10A.
%   [Q, PE, RE] = IK(Ttarget, T6tcp) solves a desired TCP pose. Ttarget and
%   T6tcp are 4-by-4 homogeneous transforms; translations use millimetres.
%   T6tcp is the fixed TCP transform relative to frame 6 and defaults to eye(4).
%   Q contains rows [q1 ... q6] in radians. PE is in mm and RE is in radians.
%
%   q1..q3 are solved from the planar DH geometry. q4..q6 are extracted from
%   R36 = R03' * R06 using both wrist branches. Since d5=125 mm translates
%   along z4, its position depends on q4; arm and wrist branches are iterated
%   to consistency. This is a matrix/trigonometric branch method, not a Jacobian.

    validateattributes(Ttarget, {'numeric'}, ...
        {'real','finite','size',[4 4]}, mfilename, 'Ttarget');
    if nargin < 2 || isempty(T6tcp)
        T6tcp = eye(4);
    end
    validateattributes(T6tcp, {'numeric'}, ...
        {'real','finite','size',[4 4]}, mfilename, 'T6tcp');

    Ttarget = double(Ttarget);
    T6tcp = double(T6tcp);
    T06 = Ttarget / T6tcp; % remove the fixed TCP transform
    R06 = T06(1:3,1:3);
    p06 = T06(1:3,4);

    % Frame-5 origin after removing d6=111.3 mm along z6.
    d6 = 111.3;
    p05 = p06 - d6 * R06(:,3);

    % Initial seeds approximate d5's projection on the a3 direction. For a
    % general q4, p05 is off the arm plane, so the seed plane check is skipped.
    q4Seeds = [-pi, -3*pi/4, -pi/2, -pi/4, 0, pi/4, pi/2, 3*pi/4];
    seeds = zeros(0,3);
    for q4Seed = q4Seeds
        seeds = [seeds; solveArmBranches(p05, 568 + 125*cos(q4Seed), false)]; %#ok<AGROW>
    end

    solutions = zeros(0,6);
    positionErrors = zeros(0,1);
    orientationErrors = zeros(0,1);
    for seedIndex = 1:size(seeds,1)
        for wristSign = [1, -1]
            q = zeros(1,6);
            q(1:3) = seeds(seedIndex,:);
            q = setWristBranch(q, R06, wristSign);

            % Iterate d5 position and the wrist orientation until q1..q3 agree.
            for iteration = 1:20
                T04 = forwardThrough(q,4);
                p04 = p05 - 125 * T04(1:3,3);
                armSolutions = solveArmBranches(p04, 568, true);
                if isempty(armSolutions)
                    break;
                end

                distances = zeros(size(armSolutions,1),1);
                for k = 1:size(armSolutions,1)
                    delta = wrapAngle(armSolutions(k,:) - q(1:3));
                    distances(k) = sum(delta.^2);
                end
                [bestDistance, bestIndex] = min(distances);
                q(1:3) = armSolutions(bestIndex,:);
                q = setWristBranch(q, R06, wristSign);
                if bestDistance < 1e-14
                    break;
                end
            end

            % Validate each candidate against the complete TCP FK.
            Tcheck = forward_kinematics(q, T6tcp);
            pe = norm(Ttarget(1:3,4) - Tcheck(1:3,4));
            re = rotationDistance(Ttarget(1:3,1:3), Tcheck(1:3,1:3));
            if isempty(solutions) || all(vecnorm(wrapAngle(solutions - q),2,2) > 1e-5)
                solutions(end+1,:) = q; %#ok<AGROW>
                positionErrors(end+1,1) = pe; %#ok<AGROW>
                orientationErrors(end+1,1) = re; %#ok<AGROW>
            end
        end
    end

    if ~isempty(solutions)
        [positionErrors, order] = sort(positionErrors, 'ascend');
        solutions = solutions(order,:);
        orientationErrors = orientationErrors(order);
    end
end

function armSolutions = solveArmBranches(targetPoint, a3Length, enforcePlane)
    % T01^-1 maps the target to frame 1. The z equation is
    % -x*sin(q1) + y*cos(q1) = d2+d3+d4 = 191 mm.
    h = 193 - 127 + 125;
    a2 = 607;
    rho = hypot(targetPoint(1), targetPoint(2));
    armSolutions = zeros(0,3);
    if rho < abs(h) - 1e-9
        return;
    end

    phi = atan2(targetPoint(2), targetPoint(1));
    offset = asin(max(-1, min(1, h/rho)));
    q1Branches = uniqueAngles([phi-offset, phi-pi+offset]);
    for q1 = q1Branches
        p14 = transformPointInverse(dhStep(1,q1), targetPoint);
        x = p14(1); y = p14(2);
        r = hypot(x,y);
        if r < 1e-9
            continue;
        end

        cosQ3 = (r^2 - a2^2 - a3Length^2) / (2*a2*a3Length);
        if abs(cosQ3) > 1 + 1e-6
            continue;
        end
        q3Magnitude = acos(max(-1, min(1, cosQ3)));
        heading = atan2(y,x);
        for q3Geometry = uniqueAngles([q3Magnitude, -q3Magnitude])
            theta2Dh = wrapAngle(heading - atan2(a3Length*sin(q3Geometry), ...
                a2 + a3Length*cos(q3Geometry)));
            q2 = wrapAngle(theta2Dh + pi/2); % DH row 2 uses q2 - 90 degrees
            p24 = transformPointInverse(dhStep(2,q2), p14);
            if enforcePlane && abs(p24(3) - (-127+125)) > 2
                continue;
            end
            q3 = wrapAngle(atan2(p24(2),p24(1)));
            armSolutions(end+1,:) = [q1, q2, q3]; %#ok<AGROW>
        end
    end
end

function q = setWristBranch(q, R06, wristSign)
    T03 = forwardThrough(q,3);
    R36 = T03(1:3,1:3)' * R06;
    beta = acos(max(-1, min(1, R36(3,3))));
    b = wristSign * beta;
    sb = sin(b);
    if abs(sb) < 1e-8
        % At wrist singularities q4 and q6 are coupled; choose q6 = 0.
        a = atan2(R36(2,1), R36(1,1));
        c = 0;
    else
        a = atan2(R36(2,3)/sb, R36(1,3)/sb);
        c = atan2(R36(3,2)/sb, -R36(3,1)/sb);
    end
    q(4) = wrapAngle(a-pi/2); % DH row 4 uses q4 + 90 degrees
    q(5) = wrapAngle(-b);
    q(6) = wrapAngle(c);
end

function T = forwardThrough(q, count)
    T = eye(4);
    for i = 1:count
        T = T * dhStep(i,q(i));
    end
end

function A = dhStep(index, q)
    dh = [ ...
          0, -pi/2,  176.5,       0;
        607,      0,  193.0, -pi/2;
        568,      0, -127.0,       0;
          0,  pi/2,  125.0,  pi/2;
          0, -pi/2,  125.0,       0;
          0,      0,  111.3,       0];
    a = dh(index,1);
    alpha = dh(index,2);
    d = dh(index,3);
    th = q + dh(index,4);
    ct = cos(th); st = sin(th);
    ca = cos(alpha); sa = sin(alpha);
    A = [ct, -st*ca,  st*sa, a*ct; ...
         st,  ct*ca, -ct*sa, a*st; ...
          0,     sa,     ca,    d; ...
          0,      0,      0,    1];
end

function pLocal = transformPointInverse(T, p)
    pLocal = T(1:3,1:3)' * (p(:) - T(1:3,4));
end

function values = uniqueAngles(values)
    values = wrapAngle(values);
    if numel(values) > 1
        keep = true(size(values));
        for i = 2:numel(values)
            if any(abs(wrapAngle(values(i) - values(1:i-1))) < 1e-9)
                keep(i) = false;
            end
        end
        values = values(keep);
    end
end

function angle = wrapAngle(angle)
    angle = atan2(sin(angle), cos(angle));
end

function errorRad = rotationDistance(R1, R2)
    cosine = max(-1, min(1, (trace(R1'*R2)-1)/2));
    errorRad = acos(cosine);
end
