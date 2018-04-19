classdef extended_kalman_filter < handle
    properties
        x
        P
        find_K
        find_Phi
        find_H
    end
    
    methods
        function obj = extended_kalman_filter(n)
            obj.x = zeros(n, 1);
            obj.P = eye(n);
        end
        
        function predict(self, f, Phi, Q)
            
            self.x = f(self.x);
            self.P = Phi * self.P * Phi' + Q;
            self.find_Phi = Phi;
        end
        
        function correct(self, z, R, h, H)
            y = z - h(self.x);
            S = H * self.P * H' + R;
            K = self.P * H' / S;
            self.x = self.x + K * y;
            n = length(self.P);
            self.P = (eye(n) - K*H) * self.P;
            self.find_K = K;
            self.find_H = H;
        end
        
        function reset(self, x, P)
            self.x = x;
            self.P = P;
        end
    end
end
        