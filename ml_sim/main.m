% MAIN Vi du goi ham dong hoc thuan.
% Chay script nay trong MATLAB khi current folder la ml_sim,
% hoac mo main.m va bam Run.

clear;
clc;

% Goc khop q1..q6 (radian). Thay cac gia tri nay theo cau hinh can tinh.
theta = [0, 0, 0, 0, 0, 0];

T06 = forward_kinematics(theta);

disp('Ma tran T06:');
disp(T06);
