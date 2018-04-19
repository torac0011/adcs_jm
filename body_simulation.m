classdef body_simulation < handle
    properties
        body
        type
        control
        b_dot
        
        gyro_bias
        delta_t
        measurement_noise
        perturbation_torque
        rate_gyro_noise
        rate_gyro_bias_noise

        find_torque = [0; 0; 0]
        find_z1 = [0; 0; 0]
        find_z2 = [0; 0; 0]
    end
    
    methods
        function obj = body_simulation(filter_model, delta_t, inertia, rate, measurement_noise, perturbation_torque,...
                rate_gyro_noise, rate_gyro_bias_noise, control_model)
            
            obj.delta_t = delta_t;
            obj.measurement_noise = measurement_noise;
            obj.perturbation_torque = perturbation_torque;
            obj.rate_gyro_noise = rate_gyro_noise;
            obj.rate_gyro_bias_noise = rate_gyro_bias_noise;

            obj.body = rotation(diag(inertia));
            obj.body.set_rate(randn(3,1) * rate); % set random rate
            obj.body.set_attitude(get_rand_quat()); % set random attitude

            gyro_init_bias = 2/180*pi; % initial gyro bias
            obj.gyro_bias = randn(3, 1) * gyro_init_bias;
            if (strcmp(filter_model, 'EKF'))
                % basic extended kalman filter
                Q = diag([ones(1, 4)*0.001^2, ones(1, 3)*0.001^2]);
                R = eye(6)*measurement_noise^2;
                x0 = [1; 0; 0; 0; zeros(3,1)];
                P0 = diag([ones(1, 4)*3^2, ones(1, 3)*10^2]);
                EKF = dynamics_numeric(delta_t, inertia, Q, R);
                EKF.Kalman.reset(x0, P0);
                obj.type = EKF;
            elseif (strcmp(filter_model, 'MEKF'))
                % const momentum multiplicative kalman filter
                Q = eye(3)*perturbation_torque^2;
                R = eye(2)*measurement_noise^2;
                MEKF = MEKF_num(delta_t, Q, R, inertia);
                P0 = diag([ones(1, 3)*1000, ones(1, 3)*rate^2]);
                MEKF.Kalman.reset(zeros(6,1), P0);
                obj.type = MEKF;               
            else
                error('invalid filter model')
            end
%             altitude = 500000; % in meters
%             obj.b_dot = b_dot(delta_t, inertia, altitude);
%             if (strcmp(control_model, 'MPC'))
%                 obj.control = mpc_numeric(delta_t, inertia, altitude, control_model);
%             elseif strcmp(control_model, 'BDOT')
%                 obj.control = b_dot(delta_t, inertia, altitude);
%             else
%                 error('invalid control model')
%             end
        end
        
        function set_kalman_ic(self, x, P)
            self.type.Kalman.reset(x, P);
        end

        function update(self)
            % rotation dynamics simulation with external torque
            disturbance_torque = randn(3,1) * self.perturbation_torque * sqrt(1/self.delta_t);
            % control torque applied
%             self.b_dot.get_bdot_params(self.body.get_rate);
%             try
                self.type.update_matrices(self.type.get_attitude, self.type.get_omega);
%             catch
%                 error('error occured while updating matrices for LQR');
% %                 self.control.get_bdot_params(self.body.get_rate);
%             end
            
            self.body.update(disturbance_torque, self.type.get_torque, self.delta_t);
            self.find_torque = disturbance_torque;

            % gyroscope simulation: white noise + bias random walk
            gyro = self.body.measure_rate(self.rate_gyro_noise * sqrt(1/self.delta_t));
            gyro = gyro + self.gyro_bias;
            self.gyro_bias = self.gyro_bias + randn(3, 1) * self.rate_gyro_bias_noise * sqrt(1/self.delta_t);

            self.type.predict(gyro);
            z1 = self.body.measure_vector([0; 0; 1], self.measurement_noise);
            z2 = self.body.measure_vector([0; 1; 0], self.measurement_noise);
            self.type.correct(z1, z2);
            self.find_z1 = z1;
            self.find_z2 = z2;
        end
    end
end