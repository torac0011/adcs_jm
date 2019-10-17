clear all, close all 
% setup simulation
delta_t = .1; % [s]
simulation_duration = 96*60*5; % [s]

inertia_kg_mm = [419, 420, 421]; % kg*mm^2, approx. values form skCUBE
inertia = inertia_kg_mm / 1e6; % kg*m^2
measurement_noise = 4/180*pi; % standard deviation of noise on vector components
perturbation_torque = 9.1e-8; % [Nm], perturbation torque (from Phase 0 CubETH ADCS, Camille Pirat)
perturbation_torque_noise = perturbation_torque / sqrt(1); % [Nm/sqrt(Hz)] assuming a change every second for white noise density
initial_rate = 20/180*pi;

rate_gyro_white_noise_deg_p_s = 0.03; % [deg/s/sqrt(Hz)]
rate_gyro_white_noise = rate_gyro_white_noise_deg_p_s/180*pi; % [rad/s/sqrt(Hz)]
rate_gyro_bias_instability_deg_p_s = 0.003; % [deg/s]
rate_gyro_bias_instability_time = 200; % [s]
rate_gyro_bias_random_walk_white_noise = (rate_gyro_bias_instability_deg_p_s/sqrt(rate_gyro_bias_instability_time))/180*pi; % [rad/s^2/sqrt(Hz)]

filter_model = 'EKF'; % one of 'EKF' or 'MEKF'
control_model = 'LQR'; %LQR BDOT
disp(filter_model);

nb_runs = 1;
att_errors = zeros(nb_runs, simulation_duration/delta_t);
rate_errors = zeros(nb_runs, simulation_duration/delta_t);

attitude_kalman = zeros(4, simulation_duration/delta_t);
rate_kalman = zeros(3, simulation_duration/delta_t);
attitude_body = zeros(4, simulation_duration/delta_t);
rate_body = zeros(3, simulation_duration/delta_t);

moment = zeros(3, simulation_duration/delta_t);
torque = zeros(3, simulation_duration/delta_t);
% field = zeros(3, simulation_duration/delta_t);
disturbance = zeros(3, simulation_duration/delta_t);

% attitude_body = zeros(4, simulation_duration/delta_t);
% rate_body = zeros(3, simulation_duration/delta_t);

for run_i = 1:nb_runs
    sim = body_simulation(filter_model, ...
                           delta_t, ...
                           inertia, ...
                           initial_rate, ...
                           measurement_noise, ...
                           perturbation_torque_noise, ...
                           rate_gyro_white_noise, ...
                           rate_gyro_bias_random_walk_white_noise, ...
                           control_model);

    idx = 1;
    for t = 0:delta_t:simulation_duration

        att_err = quat_mult(sim.body.get_attitude, quat_conj(sim.type.get_attitude));
        att_err = asin(norm(att_err(2:4)))*2;
        att_err_deg = att_err * 180 / pi;
        att_errors(run_i, idx) = att_err_deg;

        rate_err = norm(sim.body.get_rate() - sim.type.get_omega);
        rate_err_deg_per_sec = rate_err * 180 / pi;
        rate_errors(run_i, idx) = rate_err_deg_per_sec;

        attitude_kalman(:, idx) = sim.type.get_attitude;
        rate_kalman(:, idx) = sim.type.get_omega;
        
        moment(:, idx) = sim.type.get_mag_moment;
        torque(:, idx) = sim.type.get_torque;
        
%         field(:, idx) = sim.type.EMF.get_mf; %EMF
        
%         attitude_body(:, idx) = sim.body.get_attitude;
%         rate_body(:, idx) = sim.body.get_rate;
        disturbance(:, idx) = sim.find_torque;
        idx = idx+1;
        progress = idx*100/length(attitude_kalman);
        disp(progress);
        sim.update()
    end
end

time = 0:delta_t:simulation_duration;



% sample reduction
time = reduce(time,10);
attitude_kalman = reduce(attitude_kalman,10);
rate_kalman = reduce(rate_kalman, 10);
moment = reduce(moment,10);
torque = reduce(torque, 10);
disturbance = reduce(disturbance, 10);
att_errors = reduce(att_errors, 10);
rate_errors = reduce(rate_errors, 10);

figure
semilogy(time/96/60, att_errors')
hold on
plot(time/96/60, ones(length(time/96/60), 1)*5, 'k--') % requirement: 2 deg
grid on;
% title([strrep(filter_model, '_', ' ') ' attitude error [deg]'])
xlabel('time [orbit]','interpreter','latex','FontSize',8)
ylabel('attitude error [deg]','interpreter','latex','FontSize',8)
set(gca,'FontSize',20)
set(gcf, 'PaperPosition',[0 0 16 9])
% print('attitude_err','-dpng', '-r200')
tightfig
saveas(gcf, 'periodic_attitude_errors', 'epsc'); %strcat(strrep(control_model, '_', ' '),'

figure
semilogy(time/96/60, rate_errors')
hold on
plot(time/96/60, ones(length(time/96/60), 1)*3, 'k--') % requirement: 0.5 deg/s
grid on;
% title([strrep(filter_model, '_', ' ') ' rate error [deg/s]'])
xlabel('time [orbit]','interpreter','latex','FontSize',8)
ylabel('rate error [deg/s]','interpreter','latex','FontSize',8)
set(gca,'FontSize',20)
set(gcf, 'PaperPosition',[0 0 16 9])
% print('rate_err','-dpng', '-r200')
tightfig
saveas(gcf, 'periodic_rate_errors', 'epsc');

% KALMAN STATES PLOT
qb = 1.2;
q1ref = ones(1,length(time));
q1ref2 = -q1ref;
qref = zeros(1,length(time));
omegaref = qref;
figure
subplot(411)
plot(time/96/60, attitude_kalman(1,:),'k',time/96/60, q1ref, 'y--',time/96/60, q1ref2, 'y--');%,time/90/60, attitude_kalman(2,:),time/90/60, attitude_kalman(3,:),time/90/60, attitude_kalman(4,:));
grid on;
% title('Quaternions');
xlabel('Time [orbit]'); ylabel('$\mathit{\eta}$','interpreter','latex');
ylim([-qb qb]);
subplot(412)
plot(time/96/60, attitude_kalman(2,:),'r',time/96/60, qref, 'y--');
grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{q}_1$','interpreter','latex');
ylim([-qb qb]);
subplot(413)
plot(time/96/60, attitude_kalman(3,:),'g',time/96/60, qref, 'y--');
grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{q}_2$','interpreter','latex');
ylim([-qb qb]);
subplot(414)
plot(time/96/60, attitude_kalman(4,:),'b',time/96/60, qref, 'y--');
grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{q}_3$','interpreter','latex');
ylim([-qb qb]);
tightfig
saveas(gcf, 'periodic_attitude', 'epsc');
%
wb = .002;
figure
subplot(311)
plot(time/96/60, rate_kalman(1,:),'g',time/96/60, omegaref, 'y--');%,time/96/60, rate_kalman(2,:),time/96/60, rate_kalman(3,:));
grid on;
% title('Roll');
xlabel('Time [orbit]'); ylabel('$\mathit{w_1}$ [rad.s$^{-1}$]','interpreter','latex');
% ylim([-wb wb]);
subplot(312)
plot(time/96/60, rate_kalman(2,:),'r',time/96/60, omegaref, 'y--');
grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{w_2}$ [rad.s$^{-1}$]','interpreter','latex');
% ylim([-wb wb]);
% title('Pitch');
subplot(313)
plot(time/96/60, rate_kalman(3,:),'b',time/96/60, omegaref, 'y--');
grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{w_3}$ [rad.s$^{-1}$]','interpreter','latex');
% ylim([-wb wb]);
% title('Yaw');
tightfig
saveas(gcf, 'periodic_rate', 'epsc');

%Action control plot
% mb = .06;
figure
subplot(311)
plot(time/96/60, moment(1,:)); grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{m}_\mathrm{x}$ [A.m$^2$]','interpreter','latex');
% title('Magnetic moment');
% ylim([-mb mb]);
subplot(312)
plot(time/96/60, moment(2,:)); grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{m}_\mathrm{y}$ [A.m$^2$]','interpreter','latex');
% ylim([-mb mb]);
subplot(313)
stairs(time/96/60, moment(3,:)); grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{m}_\mathrm{z}$ [A.m$^2$]','interpreter','latex');
% ylim([-mb mb]);
tightfig
saveas(gcf, 'periodic_moment', 'epsc');

figure
subplot(311)
plot(time/96/60, torque(1,:)); grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{\tau}_\mathrm{c,x}$ [N.m]','interpreter','latex');
% title('Control torque');
subplot(312)
plot(time/96/60, torque(2,:)); grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{\tau}_\mathrm{c,y}$ [N.m]','interpreter','latex');
subplot(313)
plot(time/96/60, torque(3,:)); grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{\tau}_\mathrm{c,z}$ [N.m]','interpreter','latex');
tightfig
saveas(gcf, 'periodic_torque', 'epsc');

figure
subplot(311)
plot(time/96/60, disturbance(1,:));grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{\tau}_\mathrm{d,x}$ [N.m]','interpreter','latex');
% title('Disturbance torque');
subplot(312)
plot(time/96/60, disturbance(2,:));grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{\tau}_\mathrm{d,y}$ [N.m]','interpreter','latex');
subplot(313)
plot(time/96/60, disturbance(3,:));grid on;
xlabel('Time [orbit]'); ylabel('$\mathit{\tau}_\mathrm{d,z}$ [N.m]','interpreter','latex');
tightfig
saveas(gcf, 'disturbance', 'epsc');
% % Geomagnetic field plot
% bb = 5e-5;
% figure
% subplot(311)
% plot(time/96/60, field(1,:));
% title('Geomagnetic field');
% ylim([-bb bb]);
% subplot(312)
% plot(time/96/60, field(2,:));
% ylim([-bb bb]);
% subplot(313)
% plot(time/96/60, field(3,:));
% ylim([-bb bb]);

% % BODY STATES PLOT
% figure
% qb = 1.2;
% subplot(221)
% plot(time, attitude_body(1,:));
% title('BODY');
% ylim([-qb qb]);
% subplot(222)
% plot(time, attitude_body(2,:));
% ylim([-qb qb]);
% subplot(223)
% plot(time, attitude_body(3,:));
% ylim([-qb qb]);
% subplot(224)
% plot(time, attitude_body(4,:));
% ylim([-qb qb]);
% 
% figure
% wb = .5;
% subplot(311)
% plot(time, rate_body(1,:));
% title('Roll BODY');
% ylim([-wb wb]);
% subplot(312)
% plot(time, rate_body(2,:));
% ylim([-wb wb]);
% title('Pitch');
% subplot(313)
% plot(time, rate_body(3,:));
% ylim([-wb wb]);
% title('Yaw');

converged_time = 60; % s
% 
mean_rate_errors = mean(rate_errors(:, converged_time/delta_t:end)')
mean(mean_rate_errors);
std_rate_errors = std(rate_errors(:, converged_time/delta_t:end)')
mean(std_rate_errors);
max_rate_errors = max(rate_errors(:, converged_time/delta_t:end)')
max(max_rate_errors);

mean_att_errors = mean(att_errors(:, converged_time/delta_t:end)')
mean(mean_att_errors);
std_att_errors = std(att_errors(:, converged_time/delta_t:end)')
mean(std_att_errors);
max_att_errors = max(att_errors(:, converged_time/delta_t:end)')
max(max_att_errors);

disp(sim.type.control.failure)
