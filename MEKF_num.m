classdef MEKF_num < handle
    properties
        delta_t
        q_ref
        Kalman
        Q
        R
        G
        F
        f
        inspect_Phi
        inspect_K
        inspect_H
        EMF
        MPC
        control_torque = [0; 0; 0]
    end

    methods
        function obj = MEKF_num(delta_t, Q, R, inertia)
            obj.Q = Q;
            obj.R = R;
            MEKF_sym;
            F__ = matlabFunction(subs(F, I, diag(inertia)), 'Vars', {w_hat});
            G__ = matlabFunction(subs(G, I, diag(inertia)), 'Vars', {B});
            obj.f = matlabFunction(subs(f_hat, I, diag(inertia)), 'Vars', {w_hat, B, M}); % parameters for ode45
            obj.F = @(x_) F__(x_(4:6));
            obj.G = @(b_) G__(b_);
            obj.Kalman = extended_kalman_filter(6);
            altitude = 500000;
            obj.EMF = b_dot(delta_t, inertia, altitude); % 3rd arg - altitude
            % covariance matrices for LQR gain
            Q = [eye(3)*2 zeros(3,3); zeros(3,3) eye(3)]; %for MEKF
            R = eye(3)*0.1;
            obj.MPC = model_predictive(delta_t, .5, .5, Q, R);
            obj.delta_t = delta_t;
            obj.q_ref = [1; 0; 0; 0];
        end

        function attitude_error_transfer_to_reference(self)
            delta_q_of_a = [2; self.Kalman.x(1); self.Kalman.x(2); self.Kalman.x(3)]; % unnormalized !
            self.q_ref = quat_mult(self.q_ref, delta_q_of_a);
            self.q_ref = self.q_ref / norm(self.q_ref); % normalize after multiplication
            self.Kalman.x(1:3) = zeros(3, 1);
        end

        function predict(self, unused_gyro)
            % propagate reference
            omega = self.Kalman.x(4:6);
            %TODO update geomagnetic field with rate DONE
            self.EMF.get_bdot_params(omega);
            ang = norm(omega) * self.delta_t;
            if ang > 0.000001
                axis = omega / norm(omega);
                delta_q_ref = [cos(ang/2); axis*sin(ang/2)];
            else
                delta_q_ref = [1; omega*self.delta_t/2];
            end
            self.q_ref = quat_mult(self.q_ref, delta_q_ref);

            F = self.F(self.Kalman.x); 
            G = self.G(self.EMF.get_mf);
            % Phi = eye(6) + self.delta_t * self.F(self.K.x);
            % Qs = self.G*self.Q*self.G' * self.delta_t;
            A = [        -F, G*self.Q*G';
                 zeros(6,6),     F'];
            B = expm(A*self.delta_t);
            Phi = B(7:12, 7:12)';
            Qs = Phi * B(1:6, 7:12);

%             % integrate kalman state
            [t, x] = ode45(@(t, x_) self.f(x_(4:6), self.EMF.get_mf, self.get_mag_moment), [0 self.delta_t], self.Kalman.x);
            x_new = x(end, :)';
            f = @(x) x_new;

            self.Kalman.predict(f, Phi, Qs)
            self.inspect_Phi = Phi;
        end
        
        function update_matrices(self, attitude, rate) % function for body_simulation update
            F = self.F(self.Kalman.x);
            G = self.G(self.EMF.get_mf);
            H = eye(6);   % 6 for MEKF   
%             self.attitude_error_transfer_to_reference;
%             disp(self.q_ref);
            x_ref = [1; zeros(1, 6)'];
%             attitude = self.q_ref;
            self.MPC.update_mpc(F, G, H, x_ref, attitude, rate);
            self.control_torque = cross(self.get_mag_moment, self.EMF.get_mf);
        end
        
        function control_torque = get_torque(self)
            control_torque = self.control_torque;
        end
        
        function moment = get_mag_moment(self)
            moment = self.MPC.get_action_control;
        end
        
        function measure_vect(self, expected_i, measured_b, R)
            expected_i = expected_i / norm(expected_i);
            measured_b = measured_b / norm(measured_b);
            i_to_m = vec2quat(expected_i, [1; 0; 0]);
            b_to_m = quat_mult(i_to_m, self.q_ref);
            b_to_m = quat2rmat(b_to_m);
            expected_b = quat_rotate(expected_i, quat_conj(self.q_ref));
            Proj = [0, 1, 0;
                    0, 0, 1];
            %z = Proj * b_to_m * measured_b; % simple linear measurement
            m = b_to_m * measured_b + [1; 0; 0];
            m = m / norm(m);
            z = m(2:3) / m(1) * 2; % improved nonlinear measurement

            h = @(x) [0; 0]; % expected measurement is zero
            Ha = Proj * b_to_m * skew(expected_b);
            H = [Ha, zeros(2, 3)];
            self.Kalman.correct(z, R, h, H)
        end


        function correct(self, E1, E2)
            self.measure_vect([0; 0; 1], E1, self.R)
            self.measure_vect([0; 1; 0], E2, self.R)
            self.attitude_error_transfer_to_reference()
        end

        function set_attitude(self, att)
            self.q_ref = att;
        end

        function att = get_attitude(self)
            att = self.q_ref;
        end

        function omega = get_omega(self)
            omega = self.Kalman.x(4:6);
        end
    end
end