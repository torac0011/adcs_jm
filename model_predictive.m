classdef model_predictive < handle
    properties
        delta_t
        top_sat % saturation
        bot_sat
        Q % state covariance
        R % inputs covariance matrix
        gain
        find_x
        find_u = [0; 0; 0]
        sat_dyn
%         control_torque
    end
    
    methods
        function obj = model_predictive(delta_t, top_sat, bot_sat, Q, R)
            % object properties declaration and initialization
            obj.delta_t = delta_t;
            obj.top_sat = top_sat;
            obj.bot_sat = bot_sat;
            obj.Q = Q;
            obj.R = R;
        end
        
        function update_mpc(self, F, G, H, x_ref, attitude, rate)
%             try
                sys = ss(F,G,H,zeros(6,3));
                if rank(ctrb(sys)) < 6 % 6 for MEKF 
                    self.find_u = zeros(3,1);
                    disp('System is uncontrollable!');
                    disp('System rank:'); disp(rank(ctrb(sys)));
                else
%                     try
                        [A, B, C, D] = c2dm(F, G, H, zeros(6,3), self.delta_t, 'zoh'); %6 for MEKF
%                     catch
%                         error('cannot convert to discrete state')
%                     end
%                     try
                        self.gain = -dlqr(A, B, self.Q, self.R); %LQ gain
%                     catch
%                         error('cannot find optimal gain')
%                     end
                    a = 2 * [attitude(2); attitude(3); attitude(4)] / attitude(1);
%                     a = a / norm(a)
                    self.find_x = [a; rate];
                    self.find_u = self.gain * (self.find_x );
                    % constraints
                    for i = 1 : length(self.find_u)
                        if self.find_u(i) > self.top_sat
                            self.find_u(i) = self.top_sat;
                        elseif self.find_u(i) < -self.bot_sat
                            self.find_u(i) = -self.bot_sat;
                        end
                    end
                end
%             catch
%                 self.find_u = zeros(3,1);
%                 disp('Method update_mpc have non consistent matrices F and G!');
%             end
        end
        
        function find_u = get_action_control(self)
            find_u = self.find_u;
        end

    end
end