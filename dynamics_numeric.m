classdef dynamics_numeric < handle
    properties
        delta_t
        Kalman
        R
        Q
        f
        Phi
        G
        h
        H
        EMF
        MPC
        control_torque = [0; 0; 0]
    end
    
    methods
        function obj = dynamics_numeric(delta_t, inertia, Q, R)
            % state vector x = [q1 q2 q3 q4 w1 w2 w3]'
            dynamics_symbolic_derivation
            f__ = matlabFunction(subs(f, [dt; I11; I22; I33], [delta_t; inertia']), 'Vars', {x, M, B});
            obj.f = @(x_, u, B) f__(x_(1), x_(2), x_(3), x_(4), x_(5), x_(6), x_(7), u(1), u(2), u(3), B(1), B(2), B(3));
            F__ = matlabFunction(subs(F, [dt; I11; I22; I33], [delta_t; inertia']), 'Vars', x);
            obj.Phi = @(x_, u) F__(x_(1), x_(2), x_(3), x_(4), x_(5), x_(6), x_(7));
            % input vector u = [M1 M2 M3]'
            G__ = matlabFunction(subs(G, [dt; I11; I22; I33], [delta_t; inertia']), 'Vars', {M, B});
            obj.G = @(u, B) G__(u, B);
            % measurement z = [z1_1, z1_2, z1_3, z2_1, z2_2, z2_3]
            h__ = matlabFunction(subs(h, dt, delta_t), 'Vars', x);
            obj.h = @(x_) h__(x_(1), x_(2), x_(3), x_(4), x_(5), x_(6), x_(7));
            H__ = matlabFunction(subs(H, dt, delta_t), 'Vars', x);
            obj.H = @(x_) H__(x_(1), x_(2), x_(3), x_(4), x_(5), x_(6), x_(7));
            obj.Q = Q;
            obj.R = R;
            obj.Kalman = extended_kalman_filter(7);
            obj.delta_t = delta_t;
            
            %TODO here goes the object definition of geomagnetic field reference DONE
            obj.EMF = b_dot(delta_t, inertia, 500000); % 3rd arg - altitude
            %TODO here goes obj defintion of model_predictive DONE
            % covariance matrices for LQR gain
            Q = [eye(4)*.5 zeros(4,3); zeros(3,4) eye(3)]; %for EKF
            R = eye(3)*1.6;
            obj.MPC = model_predictive(delta_t, .5, .5, Q, R);           
            
        end
        
        function update_matrices(self, attitude, rate) % function for body_simulation update
            F = self.Phi(self.Kalman.x);
            G = self.G(self.get_mag_moment, self.EMF.get_mf);
            H = self.H(self.Kalman.x);         
            x_ref = zeros(1, 6)';
            self.MPC.update_mpc(F, G, H, x_ref, attitude, rate);
            self.control_torque = cross(self.get_mag_moment, self.EMF.get_mf);
        end
        
        function control_torque = get_torque(self)
            control_torque = self.control_torque;
        end
        
        function moment = get_mag_moment(self)
            moment = self.MPC.get_action_control;
        end
        
        function renorm_quat(self)
            n = norm(self.Kalman.x(1:4));
            self.Kalman.x(1:4) = self.Kalman.x(1:4)/n;
        end
        
        function predict(self, unused_gyro)
            self.Kalman.predict(self.f, self.Phi(self.Kalman.x), self.Q)%, self.get_mag_moment, self.EMF.get_mf)
            self.renorm_quat
        end
        
        function correct(self, Z1, Z2)
            self.Kalman.correct([Z1; Z2], self.R, self.h, self.H(self.Kalman.x))
            self.renorm_quat
        end
        
        function att = get_attitude(self)
            att = self.Kalman.x(1:4);
        end

        function omega = get_omega(self)
            omega = self.Kalman.x(5:7);
        end
    end
end

