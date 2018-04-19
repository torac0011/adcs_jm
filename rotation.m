classdef rotation < handle
    properties
        attitude = [1; 0; 0; 0]; % attitude quaternion
        rate = [0; 0; 0]; % angular rate
        inertia
    end
    methods
        function obj = rotation(inertia)
            obj.inertia = inertia;
        end
        
        function attitude = get_attitude(self)
            attitude = self.attitude;
        end
        
        function rate = get_rate(self)
            rate = self.rate;
        end
        
        function inertia = get_inertia(self)
            inertia = self.inertia;
        end
        
        function set_attitude(self, attitude)
            self.attitude = attitude;
        end
        
        function set_rate(self, rate)
            self.rate = rate;
        end
        
        function set_inertia(self, inertia)
            self.inertia = inertia;
        end
        
        function update(self, disturbance_torque, control_torque, duration)        % FUNCTION UPDATE: CONTROL TORQUE APPLIED
            function x_dot = diff_eq(t, x)
                att = x(1:4);
                omega = x(5:7);
                q_dot = 1/2 * quat_mult(att, [0; omega]);
                I = self.inertia;
                % T = I * omega_dot + omega x I * omega
                omega_dot = I \ (control_torque + disturbance_torque - cross(omega, I * omega));
                x_dot = [q_dot; omega_dot];
            end
            x = [self.attitude; self.rate];
            [t, x] = ode45(@diff_eq, [0 duration], x);
            self.attitude = x(end, 1:4)';
            self.rate = x(end, 5:7)';
        end
        
        % measure a vector (given in the inertial frame) in the body frame with additive noise
        function vBody = measure_vector(self, vInertial, noise)
            vBody = quat_rotate(vInertial, quat_conj(self.attitude));
            vBody = vBody + randn(3, 1) * noise;
        end

        % measure the angular rate with additive noise
        function omega = measure_rate(self, noise)
            omega = self.rate + randn(3, 1) * noise;
        end
    end
end
        

        