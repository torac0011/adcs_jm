clf;
clear, close all;

% mpc_symbolic;
% setup simulation
delta_t = .1; % [s]
simulation_duration = 100*60; % [s]
speedup = 1; % 0 or 1

inertia_kg_mm = [1.92e3, 2.46e3, 2.34e3]; % kg*mm^2, approx. values form swisscube
inertia = inertia_kg_mm / 1e6; % kg*m^2
measurement_noise = 4/180*pi; % standard deviation of noise on vector components
perturbation_torque = 9.1e-8; % [Nm], perturbation torque
perturbation_torque_noise = perturbation_torque / sqrt(1); % [Nm/sqrt(Hz)] assuming a change every second for white noise density
initial_rate = 30/180*pi;

rate_gyro_noise_deg_p_s = 0.03; % [deg/s/sqrt(Hz)]
rate_gyro_noise = rate_gyro_noise_deg_p_s/180*pi; % [rad/s/sqrt(Hz)]
rate_gyro_bias_instability_deg_p_s = 0.003; % [deg/s]
rate_gyro_bias_instability_time = 200; % [s]
rate_gyro_bias_random_walk_white_noise = (rate_gyro_bias_instability_deg_p_s/sqrt(rate_gyro_bias_instability_time))/180*pi; % [rad/s^2/sqrt(Hz)]

filter_model = 'MEKF'; % one of 'EKF' or 'MEKF'
control_model = 'MPC';
disp(filter_model);

sim = body_simulation(filter_model, ...
                       delta_t, ...
                       inertia, ...
                       initial_rate, ...
                       measurement_noise, ...
                       perturbation_torque_noise, ...
                       rate_gyro_noise, ...
                       rate_gyro_bias_random_walk_white_noise, ...
                       control_model);



% setup 3D plot
figure(1)
clf
axis equal;
grid on;
xlabel('X','FontSize',18, 'Color','r');
ylabel('Y','FontSize',18, 'Color','g')
zlabel('Z','FontSize',18, 'Color','b')
h = gca;
h.XLim = [-3 9];
h.YLim = [-3 3];
h.ZLim = [-3 3];
h.XTick = -1:1:1;
h.YTick = -1:1:1;
h.ZTick = -1:1:1;
view(30,30);
body_plot = patch('FaceColor', 'flat');
estim_plot = patch('FaceColor', 'flat'); %, 'FaceAlpha',.7
ang_mom_v = patch('EdgeColor', 'c');
omega_v = patch('EdgeColor', 'r');
z1_v = patch('EdgeColor', 'b');
z2_v = patch('EdgeColor', 'b');


% Setup rate plot
hold on
rate_plt_nb_pts = 250;
figure(2)
clf
subplot(3,1,1)
roll = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
roll_estim = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r', 'LineStyle', ':');
title('roll rate')
subplot(3,1,2)
pitch = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','g');
pitch_estim = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','g', 'LineStyle', ':');
title('pitch rate')
subplot(3,1,3)
yaw = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
yaw_estim = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b', 'LineStyle', ':');
title('yaw rate')


% setup error plot
figure(3)
clf
subplot(2,1,1)
attitude_error = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
attitude_error_stddev = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
title('attitude error [deg]')
legend('error', 'standard deviation')
ylim([0, 90])
subplot(2,1,2)
rate_error = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
rate_error_stddev = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
title('rate error [deg/s]')
legend('error', 'standard deviation')
ylim([0, 50])

% Setup magnetic field plot
figure(4)
clf
subplot(3,1,1)
Bx = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
title('Bx')
subplot(3,1,2)
By = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','g');
title('By')
subplot(3,1,3)
Bz = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
title('Bz')

% Setup control torque plot
% figure(5)
% clf
% subplot(3,1,1)
% Tcx = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
% title('Control torque (roll)')
% subplot(3,1,2)
% Tcy = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','g');
% title('Control torque (pitch)')
% subplot(3,1,3)
% Tcz = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
% title('Control torque (yaw)')

% Setup control moment plot
boundary = 1.2;
figure(6)
clf
subplot(3,1,1)
Mx = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
title('Magnetic moment (roll)')
ylim([-boundary boundary]);
subplot(3,1,2)
My = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','g');
title('Magnetic moment (pitch)')
ylim([-boundary boundary]);
subplot(3,1,3)
Mz = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
title('Magnetic moment (yaw)')
ylim([-boundary boundary]);

redraw_prescaler = 1; % update plots every nth iteration
redraw_cntdwn = 0;
for t = 0:delta_t:simulation_duration
    redraw_cntdwn = redraw_cntdwn - delta_t;
    if (redraw_cntdwn <= 0)
        redraw_cntdwn = delta_t * redraw_prescaler;

        % 3D visualization update
        omega = sim.body.get_rate;
        Lb = sim.body.get_inertia * omega;
        L = quat_rotate(Lb, sim.body.get_attitude);
        ang_mom_v.set('XData', [0, L(1)], 'YData', [0, L(2)], 'ZData', [0, L(3)]);

        omega_i = quat_rotate(omega, sim.body.get_attitude);
        omega_v.set('XData', [0, omega_i(1)], 'YData', [0, omega_i(2)], 'ZData', [0, omega_i(3)]);
        z1 = 10*quat_rotate(sim.find_z1, sim.type.get_attitude);
        z2 = 10*quat_rotate(sim.find_z2, sim.type.get_attitude);
        z1_v.set('XData', [0, z1(1)] + 6, 'YData', [0, z1(2)], 'ZData', [0, z1(3)]);
        z2_v.set('XData', [0, z2(1)] + 6, 'YData', [0, z2(2)], 'ZData', [0, z2(3)]);

        cube_plot(body_plot,[0,0,0],1000*inertia(1),1000*inertia(2),1000*inertia(3), sim.body.get_attitude);
        cube_plot(estim_plot,[6,0,0],1000*inertia(1),1000*inertia(2),1000*inertia(3), sim.type.get_attitude);

        % rate plot update
        body_rate = sim.body.get_rate();
        kalman_rate = sim.type.get_omega();
        %
        addpoints(roll,t,body_rate(1));
        addpoints(roll_estim,t,kalman_rate(1));
        addpoints(pitch,t,body_rate(2));
        addpoints(pitch_estim,t,kalman_rate(2));
        addpoints(yaw,t,body_rate(3));
        addpoints(yaw_estim,t,kalman_rate(3));

        % error plot update
        state_var = diag(sim.type.Kalman.P);
        rate_err = norm(sim.body.get_rate() - sim.type.get_omega);
        rate_err_deg_per_sec = rate_err * 180 / pi;
        att_err = quat_mult(sim.body.get_attitude, quat_conj(sim.type.get_attitude));
        att_err = asin(norm(att_err(2:4)))*2;
        att_err_deg = att_err * 180 / pi;
        addpoints(attitude_error,t, att_err_deg);
        att_err_stddev = sqrt(sum(state_var(1:3)));
        att_err_stddev_deg = att_err_stddev * 180 / pi;
        addpoints(attitude_error_stddev,t, att_err_stddev_deg);
        addpoints(rate_error,t, rate_err_deg_per_sec);
        rate_err_stddev = sqrt(sum(state_var(4:6)));
        rate_err_stddev_deg_per_sec = rate_err_stddev * 180 / pi;
        addpoints(rate_error_stddev,t, rate_err_stddev_deg_per_sec);

        % magnetic field update
        magnetic_field = sim.type.EMF.get_mf;
        %
        addpoints(Bx,t,magnetic_field(1));
        addpoints(By,t,magnetic_field(2));
        addpoints(Bz,t,magnetic_field(3));
        
%         % control torque update
%         control_torque = sim.control.get_torque;
%         %
%         addpoints(Tcx,t,control_torque(1));
%         addpoints(Tcy,t,control_torque(2));
%         addpoints(Tcz,t,control_torque(3));
        
        % magnetic moment update
        mag_moment = sim.type.get_mag_moment;
        %
        addpoints(Mx,t,mag_moment(1));
        addpoints(My,t,mag_moment(2));
        addpoints(Mz,t,mag_moment(3));
        
        
        
        
        drawnow limitrate
    end
    if (speedup == 0)
        pause(delta_t);
    end
    sim.update()
end
