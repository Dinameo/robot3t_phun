%BUILD_SIMULINK_FK Build and run a Simulink FK trajectory demo.
% Run this script from MATLAB. It creates fk_simulink_test.slx and saves
% fk_test_plot.png in the same folder.

folder = fileparts(mfilename('fullpath'));
modelName = 'fk_simulink_test';
modelFile = fullfile(folder, [modelName '.slx']);
plotFile = fullfile(folder, 'fk_test_plot.png');

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist(modelFile, 'file')
    delete(modelFile);
end

load_system('simulink');
new_system(modelName);

% Six smooth joint trajectories in radians.
qBias = deg2rad([0, 20, -30, 0, 45, 0]);
qAmplitude = deg2rad([10, 15, 12, 20, 15, 30]);
qFrequency = [0.50, 0.40, 0.35, 0.30, 0.25, 0.20];
qPhase = [0, pi/6, pi/3, pi/2, 2*pi/3, 5*pi/6];

for joint = 1:6
    block = sprintf('%s/Joint%d', modelName, joint);
    add_block('simulink/Sources/Sine Wave', block, ...
        'Position', [35, 35 + 65*(joint-1), 105, 65 + 65*(joint-1)], ...
        'Amplitude', num2str(qAmplitude(joint), 16), ...
        'Bias', num2str(qBias(joint), 16), ...
        'Frequency', num2str(qFrequency(joint), 16), ...
        'Phase', num2str(qPhase(joint), 16));
end

add_block('simulink/Signal Routing/Mux', [modelName '/JointVector'], ...
    'Inputs', '6', 'Position', [150, 120, 155, 245]);
fkBlock = [modelName '/ForwardKinematics'];
add_block('simulink/User-Defined Functions/MATLAB Function', fkBlock, ...
    'Position', [225, 145, 390, 220]);

chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', fkBlock);
chart.Script = sprintf([ ...
    'function p = fkPosition(q)\n' ...
    '%%#codegen\n' ...
    'a = [0; 607; 568; 0; 0; 0];\n' ...
    'alpha = [-pi/2; 0; 0; pi/2; -pi/2; 0];\n' ...
    'd = [176.5; 193; -127; 125; 125; 111.3];\n' ...
    'offset = [0; -pi/2; 0; pi/2; 0; 0];\n' ...
    'T = eye(4);\n' ...
    'for k = 1:6\n' ...
    '    th = q(k) + offset(k);\n' ...
    '    ct = cos(th); st = sin(th);\n' ...
    '    ca = cos(alpha(k)); sa = sin(alpha(k));\n' ...
    '    A = [ct, -st*ca, st*sa, a(k)*ct; ...\n' ...
    '         st,  ct*ca,-ct*sa, a(k)*st; ...\n' ...
    '          0,     sa,    ca, d(k); ...\n' ...
    '          0,      0,     0, 1];\n' ...
    '    T = T * A;\n' ...
    'end\n' ...
    'p = T(1:3,4);\n' ...
    'end\n']);

add_block('simulink/Sinks/Scope', [modelName '/PositionScope'], ...
    'Position', [485, 125, 540, 195]);
add_block('simulink/Sinks/To Workspace', [modelName '/PositionToWorkspace'], ...
    'VariableName', 'position_ts', 'SaveFormat', 'Timeseries', ...
    'Position', [485, 215, 585, 250]);
add_block('simulink/Sinks/To Workspace', [modelName '/JointToWorkspace'], ...
    'VariableName', 'joint_ts', 'SaveFormat', 'Timeseries', ...
    'Position', [225, 285, 325, 320]);

for joint = 1:6
    add_line(modelName, sprintf('Joint%d/1', joint), ...
        sprintf('JointVector/%d', joint), 'autorouting', 'on');
end
add_line(modelName, 'JointVector/1', 'ForwardKinematics/1', 'autorouting', 'on');
add_line(modelName, 'ForwardKinematics/1', 'PositionScope/1', 'autorouting', 'on');
add_line(modelName, 'ForwardKinematics/1', 'PositionToWorkspace/1', 'autorouting', 'on');
add_line(modelName, 'JointVector/1', 'JointToWorkspace/1', 'autorouting', 'on');

set_param(modelName, ...
    'StopTime', '10', ...
    'Solver', 'ode4', ...
    'FixedStep', '0.01', ...
    'ReturnWorkspaceOutputs', 'on');
save_system(modelName, modelFile);

simOut = sim(modelName);
positionData = simOut.get('position_ts');
jointData = simOut.get('joint_ts');
t = positionData.Time;
p = squeeze(positionData.Data);
q = squeeze(jointData.Data);
if size(p, 1) == 3 && size(p, 2) ~= 3
    p = p.';
end
if size(q, 1) == 6 && size(q, 2) ~= 6
    q = q.';
end

fig = figure('Color', 'w', 'Name', 'Simulink FK test');
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
plot(t, p(:,1), 'LineWidth', 1.2); grid on;
xlabel('Time (s)'); ylabel('X (mm)'); title('End-effector X');
nexttile;
plot(t, p(:,2), 'LineWidth', 1.2); grid on;
xlabel('Time (s)'); ylabel('Y (mm)'); title('End-effector Y');
nexttile;
plot(t, p(:,3), 'LineWidth', 1.2); grid on;
xlabel('Time (s)'); ylabel('Z (mm)'); title('End-effector Z');
nexttile;
plot3(p(:,1), p(:,2), p(:,3), 'LineWidth', 1.5); grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('End-effector trajectory'); view(3);
exportgraphics(fig, plotFile, 'Resolution', 160);

fprintf('Simulink model saved: %s\n', modelFile);
fprintf('FK trajectory plot saved: %s\n', plotFile);
fprintf('Joint trajectory data size: %d samples x %d joints\n', size(q,1), size(q,2));
