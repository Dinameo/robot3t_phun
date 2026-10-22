function robot_kinematics_app
%ROBOT_KINEMATICS_APP Interactive FK/IK UI for the DH model in this folder.
% Joint angles and RPY inputs are in degrees; position is in millimetres.

    addpath(fileparts(mfilename('fullpath')));

    q0Deg = [0, 20, -30, 0, 45, 0];
    lastSolutions = zeros(0, 6);
    T6tcp = eye(4);
    % Match the SampleScene tcp Transform: local position +Z 0.1 m and
    % local rotation -90 deg about Y. Position is expressed in DH millimetres.
    T6tcp(1:3,1:3) = [0, 0, -1; 0, 1, 0; 1, 0, 0];
    T6tcp(3,4) = 100;

    fig = uifigure('Name', 'Robot FK / IK', ...
        'Position', [100, 100, 1120, 690]);
    root = uigridlayout(fig, [1, 2]);
    root.ColumnWidth = {'1x', '1.25x'};
    root.RowHeight = {'1x'};
    root.Padding = [12, 12, 12, 12];
    root.ColumnSpacing = 12;

    jointPanel = uipanel(root, 'Title', 'Joint control / Forward kinematics');
    jointPanel.Layout.Row = 1;
    jointPanel.Layout.Column = 1;
    jointGrid = uigridlayout(jointPanel, [10, 3]);
    jointGrid.ColumnWidth = {82, '1x', 90};
    jointGrid.RowHeight = [{34}, repmat({44}, 1, 6), {38, 38, 28}];
    jointGrid.Padding = [10, 8, 10, 8];
    jointGrid.RowSpacing = 5;

    titleLabel = uilabel(jointGrid, 'Text', ...
        'Adjust q1...q6. Values are degrees; FK position is mm.');
    titleLabel.FontWeight = 'bold';
    titleLabel.Layout.Row = 1;
    titleLabel.Layout.Column = [1, 3];

    qSliders = gobjects(1, 6);
    qFields = gobjects(1, 6);
    for k = 1:6
        row = k + 1;
        label = uilabel(jointGrid, 'Text', sprintf('q%d (deg)', k));
        label.Layout.Row = row;
        label.Layout.Column = 1;

        qSliders(k) = uislider(jointGrid, 'Limits', [-360, 360], ...
            'Value', q0Deg(k), 'MajorTicks', [-360, -180, 0, 180, 360]);
        qSliders(k).Layout.Row = row;
        qSliders(k).Layout.Column = 2;
        qSliders(k).ValueChangedFcn = @(src, ~) sliderChanged(k, src.Value);

        qFields(k) = uieditfield(jointGrid, 'numeric', ...
            'Limits', [-360, 360], 'Value', q0Deg(k));
        qFields(k).Layout.Row = row;
        qFields(k).Layout.Column = 3;
        qFields(k).ValueChangedFcn = @(src, ~) fieldChanged(k, src.Value);
    end

    positionLabel = uilabel(jointGrid, 'Text', 'Position:');
    positionLabel.FontWeight = 'bold';
    positionLabel.Layout.Row = 8;
    positionLabel.Layout.Column = [1, 3];
    positionValue = uilabel(jointGrid, 'Text', '');
    positionValue.Layout.Row = 9;
    positionValue.Layout.Column = [1, 3];
    rpyValue = uilabel(jointGrid, 'Text', '');
    rpyValue.Layout.Row = 10;
    rpyValue.Layout.Column = [1, 3];

    targetPanel = uipanel(root, 'Title', 'Target pose / Inverse kinematics');
    targetPanel.Layout.Row = 1;
    targetPanel.Layout.Column = 2;
    targetGrid = uigridlayout(targetPanel, [9, 4]);
    targetGrid.ColumnWidth = {75, '1x', 75, '1x'};
    targetGrid.RowHeight = {34, 38, 38, 38, 42, 30, 28, '1x', 28};
    targetGrid.Padding = [10, 8, 10, 8];
    targetGrid.RowSpacing = 5;

    targetTitle = uilabel(targetGrid, 'Text', ...
        'Enter target position (mm) and ZYX roll-pitch-yaw (deg).');
    targetTitle.FontWeight = 'bold';
    targetTitle.Layout.Row = 1;
    targetTitle.Layout.Column = [1, 4];

    poseFields = gobjects(1, 6); % [x y z roll pitch yaw]
    poseNames = {'X (mm)', 'Y (mm)', 'Z (mm)', ...
        'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'};
    poseRows = [2, 2, 3, 3, 4, 4];
    poseCols = [1, 3, 1, 3, 1, 3];
    for k = 1:6
        label = uilabel(targetGrid, 'Text', poseNames{k});
        label.Layout.Row = poseRows(k);
        label.Layout.Column = poseCols(k);
        poseFields(k) = uieditfield(targetGrid, 'numeric');
        poseFields(k).Layout.Row = poseRows(k);
        poseFields(k).Layout.Column = poseCols(k) + 1;
        if k <= 3
            poseFields(k).Limits = [-2000, 2000];
        else
            poseFields(k).Limits = [-360, 360];
        end
    end

    solveButton = uibutton(targetGrid, 'push', 'Text', 'Solve IK', ...
        'FontWeight', 'bold');
    solveButton.Layout.Row = 5;
    solveButton.Layout.Column = [1, 2];
    solveButton.ButtonPushedFcn = @(~, ~) solveTargetPose();

    flangeDownButton = uibutton(targetGrid, 'push', ...
        'Text', 'Flange down + nearest');
    flangeDownButton.Layout.Row = 5;
    flangeDownButton.Layout.Column = [3, 4];
    flangeDownButton.ButtonPushedFcn = @(~, ~) setFlangeDownTarget();

    statusLabel = uilabel(targetGrid, 'Text', 'Ready');
    statusLabel.Layout.Row = 6;
    statusLabel.Layout.Column = [1, 4];

    resultTitle = uilabel(targetGrid, 'Text', ...
        'Solutions (deg), position error (mm), orientation error (deg):');
    resultTitle.FontWeight = 'bold';
    resultTitle.Layout.Row = 7;
    resultTitle.Layout.Column = [1, 4];

    resultTable = uitable(targetGrid);
    resultTable.Layout.Row = 8;
    resultTable.Layout.Column = [1, 4];
    resultTable.ColumnName = {'q1', 'q2', 'q3', 'q4', 'q5', 'q6', ...
        'Pos err', 'Ori err'};
    resultTable.ColumnFormat = repmat({'numeric'}, 1, 8);
    resultTable.ColumnWidth = {52, 52, 52, 52, 52, 52, 68, 68};
    resultTable.CellSelectionCallback = @(~, event) selectSolution(event);

    helpLabel = uilabel(targetGrid, 'Text', ...
        'Click a solution row to load its angles into the sliders.');
    helpLabel.Layout.Row = 9;
    helpLabel.Layout.Column = [1, 4];

    % Initialize the target pose from the current joint configuration.
    updateForwardKinematics();
    initializeTargetPose();

    function sliderChanged(index, value)
        qFields(index).Value = value;
        updateForwardKinematics();
    end

    function fieldChanged(index, value)
        qSliders(index).Value = value;
        updateForwardKinematics();
    end

    function updateForwardKinematics()
        qRad = deg2rad([qFields.Value]);
        T = forward_kinematics(qRad, T6tcp);
        [roll, pitch, yaw] = rotationToRPY(T(1:3, 1:3));
        positionValue.Text = sprintf('X = %.3f   Y = %.3f   Z = %.3f mm', ...
            T(1,4), T(2,4), T(3,4));
        rpyValue.Text = sprintf('Roll = %.2f   Pitch = %.2f   Yaw = %.2f deg', ...
            roll, pitch, yaw);
    end

    function initializeTargetPose()
        qRad = deg2rad([qFields.Value]);
        T = forward_kinematics(qRad, T6tcp);
        [roll, pitch, yaw] = rotationToRPY(T(1:3, 1:3));
        targetPose = [T(1,4), T(2,4), T(3,4), roll, pitch, yaw];
        for index = 1:6
            poseFields(index).Value = targetPose(index);
        end
    end

    function solveTargetPose()
        try
            xyz = [poseFields(1).Value; poseFields(2).Value; poseFields(3).Value];
            rpy = deg2rad([poseFields(4).Value, poseFields(5).Value, poseFields(6).Value]);
            R = rpyToRotation(rpy(1), rpy(2), rpy(3));
            targetT = [R, xyz; 0, 0, 0, 1];
            [lastSolutions, pError, rError] = IK(targetT, T6tcp);
            resultTable.Data = [rad2deg(lastSolutions), pError, rad2deg(rError)];
            statusLabel.Text = sprintf('Found %d solution(s).', size(lastSolutions, 1));
        catch exception
            lastSolutions = zeros(0, 6);
            resultTable.Data = zeros(0, 8);
            statusLabel.Text = ['IK failed: ', exception.message];
        end
    end

    function setFlangeDownTarget()
        % RPY is ZYX in the DH robot frame. Point flange +Z down (-Z),
        % preserve its current yaw, then include the fixed flange-to-TCP pose.
        tcpRpy = deg2rad([poseFields(4).Value, poseFields(5).Value, poseFields(6).Value]);
        tcpRotation = rpyToRotation(tcpRpy(1), tcpRpy(2), tcpRpy(3));
        flangeRotation = tcpRotation * T6tcp(1:3,1:3)';
        [~, ~, flangeYawDeg] = rotationToRPY(flangeRotation);
        flangeDown = rpyToRotation(pi, 0, deg2rad(flangeYawDeg));
        tcpDown = flangeDown * T6tcp(1:3,1:3);
        [rollDeg, pitchDeg, yawDeg] = rotationToRPY(tcpDown);
        poseFields(4).Value = rollDeg;
        poseFields(5).Value = pitchDeg;
        poseFields(6).Value = yawDeg;
        solveTargetPose();

        if isempty(lastSolutions)
            return;
        end
        % Since all branches share the requested pose, choose the closest
        % joint branch among candidates with essentially best FK residuals.
        qCurrent = deg2rad([qFields.Value]);
        posErrors = resultTable.Data(:, 7);
        oriErrors = deg2rad(resultTable.Data(:, 8));
        pointsDown = false(size(lastSolutions, 1), 1);
        for candidate = 1:size(lastSolutions, 1)
            candidateTcp = forward_kinematics(lastSolutions(candidate,:), T6tcp);
            candidateFlangeRotation = candidateTcp(1:3,1:3) * T6tcp(1:3,1:3)';
            flangeZ = candidateFlangeRotation(:,3);
            pointsDown(candidate) = dot(flangeZ, [0; 0; -1]) >= cosd(1);
        end
        eligible = find(pointsDown & ...
                        posErrors <= min(posErrors) + 0.05 & ...
                        oriErrors <= min(oriErrors) + deg2rad(0.1));
        if isempty(eligible)
            statusLabel.Text = 'No IK branch has flange Z pointing down within 1 deg.';
            return;
        end
        travel = zeros(numel(eligible), 1);
        for row = 1:numel(eligible)
            delta = atan2(sin(lastSolutions(eligible(row),:) - qCurrent), ...
                          cos(lastSolutions(eligible(row),:) - qCurrent));
            travel(row) = sum(delta.^2);
        end
        [~, nearest] = min(travel);
        selected = eligible(nearest);
        rowOrder = [selected, setdiff(1:size(lastSolutions,1), selected, 'stable')];
        lastSolutions = lastSolutions(rowOrder,:);
        resultTable.Data = resultTable.Data(rowOrder,:);
        qDeg = rad2deg(lastSolutions(1,:));
        for index = 1:6
            qFields(index).Value = qDeg(index);
            qSliders(index).Value = qDeg(index);
        end
        updateForwardKinematics();
        statusLabel.Text = sprintf('Flange points down; nearest branch applied (%d solution(s)).', ...
            size(lastSolutions,1));
    end

    function selectSolution(event)
        if isempty(event.Indices) || isempty(lastSolutions)
            return;
        end
        selectedRow = event.Indices(1, 1);
        if selectedRow < 1 || selectedRow > size(lastSolutions, 1)
            return;
        end
        qDeg = rad2deg(lastSolutions(selectedRow, :));
        for index = 1:6
            qFields(index).Value = qDeg(index);
            qSliders(index).Value = qDeg(index);
        end
        updateForwardKinematics();
    end
end

function R = rpyToRotation(roll, pitch, yaw)
% ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    cr = cos(roll); sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw); sy = sin(yaw);
    R = [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr; ...
         sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr; ...
         -sp,   cp*sr,            cp*cr];
end

function [roll, pitch, yaw] = rotationToRPY(R)
% Extract ZYX Euler angles, returned in degrees.
    pitchRad = asin(min(1, max(-1, -R(3,1))));
    if abs(cos(pitchRad)) > 1e-8
        rollRad = atan2(R(3,2), R(3,3));
        yawRad = atan2(R(2,1), R(1,1));
    else
        rollRad = atan2(-R(2,3), R(2,2));
        yawRad = 0;
    end
    roll = rad2deg(rollRad);
    pitch = rad2deg(pitchRad);
    yaw = rad2deg(yawRad);
end
