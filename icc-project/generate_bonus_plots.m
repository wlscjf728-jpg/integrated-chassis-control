% GENERATE_BONUS_PLOTS
%
%   이 스크립트는 통합 섀시 제어(ICC) 설계에서 가산점 4개 항목을 증빙하기 위해
%   시뮬레이션을 수행하고 비교 그래프(PNG)를 생성합니다.
%
%   증빙 대상 가산점 항목:
%     1. A2 Severe DLC (Moose test) 추가 통과 및 안정성 증빙
%     2. WLS 하중 배분 및 마찰원(Friction Circle) 제한 성능 증빙
%     3. C1 Single Bump에서의 CDC(반능동 서스펜션) 수직 승차감 개선 증빙
%     4. AFS 피드백 제어의 속도 의존성 Gain Scheduling 안정성 증빙
%
%   결과 파일 저장 경로:
%     G:\My Drive\Drive\Assignment\26-1\자동제어\기말프로젝트\70\integrated-chassis-control\icc-project\docs\figures/

clear; clc; close all;

% 1. 경로 설정 및 프로젝트 초기화
thisDir = fileparts(mfilename('fullpath'));
projectRoot = fullfile(thisDir, 'integrated-chassis-control', 'icc-project');
if ~exist(projectRoot, 'dir')
    error('프로젝트 폴더를 찾을 수 없습니다. 경로를 확인하십시오: %s', projectRoot);
end

cd(projectRoot);
run(fullfile(projectRoot, 'scripts', 'utils', 'init_project.m'));
addpath(fullfile(projectRoot, 'scripts'));
addpath(fullfile(projectRoot, 'scripts', 'control'));
addpath(fullfile(projectRoot, 'scripts', 'plant'));
addpath(fullfile(projectRoot, 'scripts', 'driver'));
addpath(fullfile(projectRoot, 'scripts', 'scenarios'));
addpath(fullfile(projectRoot, 'scripts', 'utils'));

% 출력 디렉토리 생성
figDir = fullfile(projectRoot, 'docs', 'figures');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end

fprintf('\n=================================================================\n');
fprintf('  ICC 가산점 증빙용 시뮬레이션 및 플롯 생성 시작\n');
fprintf('  결과 저장 경로: %s\n', figDir);
fprintf('=================================================================\n');

%% =================================================================
%% 가산점 1: A2 Severe DLC (Moose Test) 통과 및 안정성 증빙
%% =================================================================
fprintf('\n[가산점 1] A2 Severe DLC 시뮬레이션 실행 중...\n');
[res_a2_off, ~] = run_icc_scenario('A2', '14dof', 'Controller', 'off', 'SavePlot', false);
[res_a2_on,  ~] = run_icc_scenario('A2', '14dof', 'Controller', 'on',  'SavePlot', false);

fig1 = figure('Name', 'Bonus 1: A2 Severe DLC', 'Position', [100 100 1200 800], 'Color', 'w');

% 1-1. Trajectory (x-y)
subplot(2, 2, 1); hold on; grid on; box on;
plot(res_a2_off.x_pos, res_a2_off.y_pos, 'r--', 'LineWidth', 1.5);
plot(res_a2_on.x_pos, res_a2_on.y_pos, 'b-', 'LineWidth', 2.0);
if isfield(res_a2_on.scenario, 'refPath')
    plot(res_a2_on.scenario.refPath(:,1), res_a2_on.scenario.refPath(:,2), 'k:', 'LineWidth', 1.2);
    legend('Controller OFF', 'Controller ON', 'Ref Path', 'Location', 'best');
else
    legend('Controller OFF', 'Controller ON', 'Location', 'best');
end
xlabel('x [m]'); ylabel('y [m]');
title('Vehicle Trajectory (x-y)');
axis equal;

% 1-2. Sideslip Angle (beta)
subplot(2, 2, 2); hold on; grid on; box on;
plot(res_a2_off.t, rad2deg(res_a2_off.slipAngle), 'r--', 'LineWidth', 1.5);
plot(res_a2_on.t, rad2deg(res_a2_on.slipAngle), 'b-', 'LineWidth', 1.8);
yline([-2.5 2.5], 'k:', 'Limit (2.5°)', 'LineWidth', 1.2, 'Color', [0.4 0.4 0.4]);
xlabel('Time [s]'); ylabel('Sideslip angle \beta [deg]');
title('Body Sideslip Angle');
legend('Controller OFF', 'Controller ON', 'Location', 'best');

% 1-3. Yaw Rate (r)
subplot(2, 2, 3); hold on; grid on; box on;
plot(res_a2_off.t, rad2deg(res_a2_off.yawRate), 'r--', 'LineWidth', 1.5);
plot(res_a2_on.t, rad2deg(res_a2_on.yawRate), 'b-', 'LineWidth', 1.8);
plot(res_a2_on.t, rad2deg(res_a2_on.yawRateRef), 'k:', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Yaw rate r [deg/s]');
title('Yaw Rate Response');
legend('Controller OFF (Actual)', 'Controller ON (Actual)', 'Reference', 'Location', 'best');

% 1-4. Lateral Acceleration (ay)
subplot(2, 2, 4); hold on; grid on; box on;
plot(res_a2_off.t, res_a2_off.ay, 'r--', 'LineWidth', 1.5);
plot(res_a2_on.t, res_a2_on.ay, 'b-', 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Lateral acceleration a_y [m/s^2]');
title('Lateral Acceleration');
legend('Controller OFF', 'Controller ON', 'Location', 'best');

sgtitle('Bonus 1: ISO 3888-2 Severe DLC (Moose Test) Verification', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(figDir, 'bonus_a2_severe_dlc.png'));
fprintf('  -> 저장 완료: bonus_a2_severe_dlc.png\n');


%% =================================================================
%% 가산점 2: WLS 하중 배분 및 마찰원(Friction Circle) 제한
%% =================================================================
fprintf('\n[가산점 2] D1 DLC + Brake 시뮬레이션 실행 중...\n');
[res_d1_off, ~] = run_icc_scenario('D1', '14dof', 'Controller', 'off', 'SavePlot', false);
[res_d1_on,  ~] = run_icc_scenario('D1', '14dof', 'Controller', 'on',  'SavePlot', false);

fig2 = figure('Name', 'Bonus 2: Friction Circle & WLS', 'Position', [150 150 1200 800], 'Color', 'w');

% 2-1. FL Wheel Friction Circle (Fy vs Fx)
subplot(2, 2, 1); hold on; grid on; box on;
mu = 1.0;
t_circle = linspace(0, 2*pi, 100);

% Control ON - FL tire force trace
Fx_FL_on = res_d1_on.tire.FL.Fx;
Fy_FL_on = res_d1_on.tire.FL.Fy;
Fz_FL_on = res_d1_on.tire.FL.Fz;

% Control OFF - FL tire force trace
Fx_FL_off = res_d1_off.tire.FL.Fx;
Fy_FL_off = res_d1_off.tire.FL.Fy;
Fz_FL_off = res_d1_off.tire.FL.Fz;

% Plot friction circles at nominal vertical load (approx. static load ~4500N)
plot(4500 * cos(t_circle), 4500 * sin(t_circle), 'k:', 'LineWidth', 1.2);
plot(Fx_FL_off, Fy_FL_off, 'r.', 'MarkerSize', 8);
plot(Fx_FL_on, Fy_FL_on, 'b.', 'MarkerSize', 8);
xlabel('Longitudinal Force F_x [N]'); ylabel('Lateral Force F_y [N]');
title('FL Tire Force Trace (F_x - F_y) vs Friction Circle (~4.5kN)');
legend('Static Friction Limit', 'Controller OFF', 'Controller ON', 'Location', 'best');
axis equal;

% 2-2. FL Wheel Tire Utilization
% Utilization U = sqrt(Fx^2 + Fy^2) / (mu * Fz)
util_off = sqrt(res_d1_off.tire.FL.Fx.^2 + res_d1_off.tire.FL.Fy.^2) ./ max(1.0 * res_d1_off.tire.FL.Fz, 100);
util_on  = sqrt(res_d1_on.tire.FL.Fx.^2 + res_d1_on.tire.FL.Fy.^2) ./ max(1.0 * res_d1_on.tire.FL.Fz, 100);

subplot(2, 2, 2); hold on; grid on; box on;
plot(res_d1_off.t, util_off, 'r--', 'LineWidth', 1.5);
plot(res_d1_on.t, util_on, 'b-', 'LineWidth', 1.8);
yline(1.0, 'k-', 'Friction Limit (1.0)', 'LineWidth', 1.2, 'Color', 'r');
xlabel('Time [s]'); ylabel('Tire Force Utilization [-]');
title('FL Tire Utilization Rate \mu_{used} / \mu_{max}');
legend('Controller OFF', 'Controller ON', 'Location', 'best');
ylim([0 1.3]);

% 2-3. Normal Loads Fz (Dynamic Load Transfer)
subplot(2, 2, 3); hold on; grid on; box on;
plot(res_d1_on.t, res_d1_on.tire.FL.Fz, 'b-', 'LineWidth', 1.5);
plot(res_d1_on.t, res_d1_on.tire.FR.Fz, 'c-', 'LineWidth', 1.5);
plot(res_d1_on.t, res_d1_on.tire.RL.Fz, 'g-', 'LineWidth', 1.5);
plot(res_d1_on.t, res_d1_on.tire.RR.Fz, 'm-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Vertical Load F_z [N]');
title('Dynamic Vertical Loads F_z (Control ON)');
legend('FL (Front Left)', 'FR (Front Right)', 'RL (Rear Left)', 'RR (Rear Right)', 'Location', 'best');

% 2-4. Brake Torque Allocation (WLS demonstration)
subplot(2, 2, 4); hold on; grid on; box on;
plot(res_d1_on.t, abs(res_d1_on.tire.FL.Fx), 'b-', 'LineWidth', 1.5);
plot(res_d1_on.t, abs(res_d1_on.tire.FR.Fx), 'c-', 'LineWidth', 1.5);
plot(res_d1_on.t, abs(res_d1_on.tire.RL.Fx), 'g-', 'LineWidth', 1.5);
plot(res_d1_on.t, abs(res_d1_on.tire.RR.Fx), 'm-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Braking Force |F_x| [N]');
title('Braking Force Allocation (WLS proportional to F_z)');
legend('FL F_x', 'FR F_x', 'RL F_x', 'RR F_x', 'Location', 'best');

sgtitle('Bonus 2: WLS Allocation & Friction Circle Constraint Verification (D1)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig2, fullfile(figDir, 'bonus_friction_circle_wls.png'));
fprintf('  -> 저장 완료: bonus_friction_circle_wls.png\n');


%% =================================================================
%% 가산점 3: CDC Active suspension 수직 승차감 개선
%% =================================================================
fprintf('\n[가산점 3] C1 Single Bump 시뮬레이션 실행 중...\n');
[res_c1_off, ~] = run_icc_scenario('C1', '14dof', 'Controller', 'off', 'SavePlot', false);
[res_c1_on,  ~] = run_icc_scenario('C1', '14dof', 'Controller', 'on',  'SavePlot', false);

fig3 = figure('Name', 'Bonus 3: CDC Active Suspension', 'Position', [200 200 1200 800], 'Color', 'w');

% 3-1. Suspension Travel (zs) FL
subplot(2, 2, 1); hold on; grid on; box on;
plot(res_c1_off.t, res_c1_off.zs(:, 1) * 1000, 'r--', 'LineWidth', 1.5);
plot(res_c1_on.t, res_c1_on.zs(:, 1) * 1000, 'b-', 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Suspension Displacement z_s [mm]');
title('FL Suspension Travel (zs\_FL)');
legend('Controller OFF (Passive)', 'Controller ON (CDC Skyhook)', 'Location', 'best');

% 3-2. Pitch Angle (theta)
subplot(2, 2, 2); hold on; grid on; box on;
plot(res_c1_off.t, rad2deg(res_c1_off.pitch), 'r--', 'LineWidth', 1.5);
plot(res_c1_on.t, rad2deg(res_c1_on.pitch), 'b-', 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Pitch angle \theta [deg]');
title('Sprung Mass Pitch Angle');
legend('Controller OFF', 'Controller ON', 'Location', 'best');

% 3-3. Pitch Rate
subplot(2, 2, 3); hold on; grid on; box on;
dt = res_c1_on.t(2) - res_c1_on.t(1);
pitchRate_off = diff(res_c1_off.pitch) / dt;
pitchRate_on  = diff(res_c1_on.pitch) / dt;
plot(res_c1_off.t(1:end-1), rad2deg(pitchRate_off), 'r--', 'LineWidth', 1.5);
plot(res_c1_on.t(1:end-1), rad2deg(pitchRate_on), 'b-', 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Pitch rate \theta'' [deg/s]');
title('Sprung Mass Pitch Rate');
legend('Controller OFF', 'Controller ON', 'Location', 'best');

% 3-4. Damping Coefficient Comparison (Displacement RMS)
subplot(2, 2, 4); hold on; grid on; box on;
rms_off = rms(res_c1_off.zs(:,1)*1000);
rms_on  = rms(res_c1_on.zs(:,1)*1000);
bar_data = [rms_off, rms_on];
bar(1:2, bar_data, 0.5, 'FaceColor', [0.2 0.6 0.8]);
set(gca, 'XTick', 1:2, 'XTickLabel', {'Passive (OFF)', 'CDC Skyhook (ON)'});
ylabel('zs RMS [mm]');
title('FL Suspension Travel RMS Comparison');
text(1, rms_off*0.5, sprintf('%.2f mm', rms_off), 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'w');
text(2, rms_on*0.5, sprintf('%.2f mm (%.1f%% reduction)', rms_on, (1-rms_on/rms_off)*100), 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'w');

sgtitle('Bonus 3: CDC Skyhook Semi-Active Damping Ride Comfort Verification (C1)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig3, fullfile(figDir, 'bonus_cdc_ride_comfort.png'));
fprintf('  -> 저장 완료: bonus_cdc_ride_comfort.png\n');


%% =================================================================
%% 가산점 4: AFS Gain Scheduling (속도 적응 제어)
%% =================================================================
fprintf('\n[가산점 4] AFS Gain Scheduling 분석 중...\n');

fig4 = figure('Name', 'Bonus 4: Gain Scheduling', 'Position', [250 250 1200 800], 'Color', 'w');

% 4-1. Gain scheduling scaling factor vs Speed Vx
vx_range = linspace(0, 45, 100); % 0 to 162 km/h
f_gain_vx = ones(size(vx_range));
for i = 1:numel(vx_range)
    v = vx_range(i);
    if v > 20.0
        f_gain_vx(i) = max(0.3, 1.0 - 0.04 * (v - 20.0));
    end
end

subplot(2, 2, 1); hold on; grid on; box on;
plot(vx_range * 3.6, f_gain_vx, 'b-', 'LineWidth', 2.0);
xline(20.0 * 3.6, 'k--', 'Scheduling Threshold (72 km/h)', 'LineWidth', 1.2);
xlabel('Vehicle Speed V_x [km/h]'); ylabel('Gain Scaling Factor f_{gain,vx}');
title('AFS Feedback Gain Reduction Curve');
ylim([0 1.1]);

% 4-2. Active Steering Gain (Kp_afs) vs Speed Vx
Kp_base = CTRL.LAT.Kp * 0.55; % standard factor in code
plot_Kp = Kp_base * f_gain_vx;
subplot(2, 2, 2); hold on; grid on; box on;
plot(vx_range * 3.6, plot_Kp, 'm-', 'LineWidth', 2.0);
xlabel('Vehicle Speed V_x [km/h]'); ylabel('Kp_{afs} Gain Value');
title('Scheduled AFS Proportional Feedback Gain Kp_{afs}');

% 4-3. A3 Step Steer Response (ON vs OFF) at 80 km/h (High-speed stability)
[res_a3_off, ~] = run_icc_scenario('A3', '14dof', 'Controller', 'off', 'SavePlot', false);
[res_a3_on,  ~] = run_icc_scenario('A3', '14dof', 'Controller', 'on',  'SavePlot', false);

subplot(2, 2, 3); hold on; grid on; box on;
plot(res_a3_off.t, rad2deg(res_a3_off.yawRate), 'r--', 'LineWidth', 1.5);
plot(res_a3_on.t, rad2deg(res_a3_on.yawRate), 'b-', 'LineWidth', 1.8);
plot(res_a3_on.t, rad2deg(res_a3_on.yawRateRef), 'k:', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Yaw rate [deg/s]');
title('A3 High-Speed Yaw Rate Tracking (80 km/h)');
legend('Controller OFF', 'Controller ON', 'Reference', 'Location', 'best');

% 4-4. A3 Steering (Driver steer + AFS steer)
subplot(2, 2, 4); hold on; grid on; box on;
plot(res_a3_on.t, rad2deg(res_a3_on.steerDriver), 'k:', 'LineWidth', 1.5);
plot(res_a3_on.t, rad2deg(res_a3_on.steerAFS), 'b-', 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Steering Angle [deg]');
title('Driver Steer vs AFS Supplementary Steer (Control ON)');
legend('Driver Steer \delta_{driver}', 'AFS Steer \delta_{AFS}', 'Location', 'best');

sgtitle('Bonus 4: Speed-Adaptive AFS Gain Scheduling Verification (A3)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig4, fullfile(figDir, 'bonus_gain_scheduling.png'));
fprintf('  -> 저장 완료: bonus_gain_scheduling.png\n');

fprintf('\n=================================================================\n');
fprintf('  모든 가산점 증빙용 플롯 생성 완료!\n');
fprintf('=================================================================\n');
