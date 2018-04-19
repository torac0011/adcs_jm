clear, close all
% setup simulation
delta_t = .1; % [s]
simulation_duration = 96*60*1.5; % [s]

inertia_kg_mm = [1.92e3, 2.46e3, 2.34e3]; % kg*mm^2, approx. values form skCUBE
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

filter_model = 'MEKF'; % one of 'EKF' or 'MEKF'
control_model = 'MPC';
disp(filter_model);

nb_runs = 1;
att_errors = zeros(nb_runs, simulation_duration/delta_t);
rate_errors = zeros(nb_runs, simulation_duration/delta_t);

attitude_kalman = zeros(4, simulation_duration/delta_t);
rate_kalman = zeros(3, simulation_duration/delta_t);

moment = zeros(3, simulation_duration/delta_t);
torque = zeros(3, simulation_duration/delta_t);

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
        
%         attitude_body(:, idx) = sim.body.get_attitude;
%         rate_body(:, idx) = sim.body.get_rate;
        
        idx = idx+1
        sim.update()
    end
end

time = 0:delta_t:simulation_duration;




figure
semilogy(time, att_errors')
hold on
plot(time, ones(length(time), 1)*2, 'k--') % requirement: 2 deg
title([strrep(filter_model, '_', ' ') ' attitude error [deg]'])
xlabel('time [s]')
ylabel('attitude error [deg]')
set(gca,'FontSize',20)
set(gcf, 'PaperPosition',[0 0 16 9])
% print('attitude_err','-dpng', '-r200')

figure
semilogy(time, rate_errors')
hold on
plot(time, ones(length(time), 1)*0.5, 'k--') % requirement: 0.5 deg/s
title([strrep(filter_model, '_', ' ') ' rate error [deg/s]'])
xlabel('time [s]')
ylabel('rate error [deg/s]')
set(gca,'FontSize',20)
set(gcf, 'PaperPosition',[0 0 16 9])
% print('rate_err','-dpng', '-r200')

% KALMAN STATES PLOT
qb = 1.2;
figure
subplot(221)
plot(time, attitude_kalman(1,:));
title('KALMAN');
ylim([-qb qb]);
subplot(222)
plot(time, attitude_kalman(2,:));
ylim([-qb qb]);
subplot(223)
plot(time, attitude_kalman(3,:));
ylim([-qb qb]);
subplot(224)
plot(time, attitude_kalman(4,:));
ylim([-qb qb]);
%
wb = .5;
figure
subplot(311)
plot(time, rate_kalman(1,:));
title('Roll KALMAN');
ylim([-wb wb]);
subplot(312)
plot(time, rate_kalman(2,:));
ylim([-wb wb]);
title('Pitch');
subplot(313)
plot(time, rate_kalman(3,:));
ylim([-wb wb]);
title('Yaw');

%Action control plot
mb = .6;
figure
subplot(311)
stairs(time, moment(1,:));
title('Magnetic moment');
ylim([-mb mb]);
subplot(312)
stairs(time, moment(2,:));
ylim([-mb mb]);
subplot(313)
stairs(time, moment(3,:));
ylim([-mb mb]);
figure
subplot(311)
plot(time, torque(1,:));
title('Control torque');
subplot(312)
plot(time, torque(2,:));
subplot(313)
plot(time, torque(3,:));

% BODY STATES PLOT
% figure
% subplot(221)
% plot(time, attitude_body(1,:));
% title('BODY');
% subplot(222)
% plot(time, attitude_body(2,:));
% subplot(223)
% plot(time, attitude_body(3,:));
% subplot(224)
% plot(time, attitude_body(4,:));
% figure
% subplot(311)
% plot(time, rate_body(1,:));
% title('Roll BODY');
% subplot(312)
% plot(time, rate_body(2,:));
% title('Pitch');
% subplot(313)
% plot(time, rate_body(3,:));
% title('Yaw');

converged_time = 60; % s
% 
% mean_rate_errors = mean(rate_errors(:, converged_time/delta_t:end)')
% mean(mean_rate_errors)
% std_rate_errors = std(rate_errors(:, converged_time/delta_t:end)')
% mean(std_rate_errors)
% max_rate_errors = max(rate_errors(:, converged_time/delta_t:end)')
% max(max_rate_errors)
% 
% mean_att_errors = mean(att_errors(:, converged_time/delta_t:end)')
% mean(mean_att_errors)
% std_att_errors = std(att_errors(:, converged_time/delta_t:end)')
% mean(std_att_errors)
% max_att_errors = max(att_errors(:, converged_time/delta_t:end)')
% max(max_att_errors)
