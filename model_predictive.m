classdef model_predictive < handle
    properties
        delta_t
        top_sat % saturation
        bot_sat
        Q % state covariance
        R % inputs covariance matrix
        gain
        constant_gain
        find_x
        find_u = [0; 0; 0]
        sat_dyn
        failure = 0
        type
    end
    
    methods
        function obj = model_predictive(delta_t, top_sat, bot_sat, Q, R, type)
            % object properties declaration and initialization
            obj.delta_t = delta_t;
            obj.top_sat = top_sat;
            obj.bot_sat = bot_sat;
            obj.Q = Q;
            obj.R = R;
            obj.type = type;
            F = [1, zeros(6,1)';
                0, 1, 0, 0, delta_t/2, 0, 0;
                0, 0, 1, 0, 0, delta_t/2, 0;
                0, 0, 0, 1, 0, 0, delta_t/2;
                0, 0, 0, 6*(20*pi/180)^2*delta_t*(420-440)/390*1e-6, 1, 0, 0;
                0, 0, -6*(20*pi/180)^2*delta_t*(390-440)/420*1e-6, 0, 0, 1, 0;
                zeros(6,1)', 0];
            G = [zeros(4,3);
                0, 0.0086, -2.0777e-4;
                -0.008, 0, 0.004;
                1.8416e-4, -0.0038, 0];
            [A, B, C, D] = c2dm(F, G, eye(7), zeros(7,3), delta_t, 'zoh');
            A = minreal(A,B,C,D);
            B = B(2:end,:);
            Q = Q(2:end,2:end);
            obj.constant_gain = -dlqr(A, B, Q, R); 
        end
        
        function update_mpc(self, F, G, H, x_ref, attitude, rate)
%             try
                sys = ss(F,G,H,zeros(7,3));
                if rank(ctrb(sys)) < 7 % 6 for MEKF 
                    self.find_u = zeros(3,1);
                    disp('System is uncontrollable!');
                    disp('System rank:'); disp(rank(ctrb(sys)));
                else
                    self.find_x = [attitude; rate];%zeros(3,1)
                    
                    if (strcmp(self.type, 'constant'))
                        %reduction
                        self.find_x = self.find_x(2:end);
                        self.find_u = self.constant_gain * (self.find_x);
                    elseif strcmp(self.type, 'periodic')
                        [A, B, C, D] = c2dm(F, G, H, zeros(7,3), self.delta_t, 'zoh'); %6 for MEKF
                        try
                            self.gain = -dlqr(A, B, self.Q, self.R); %LQ gain
                        catch
                            self.gain = -dlqr(A, B, self.Q, self.R*1e3);
                            disp('Cannot find optimal gain. R matrix has changed')
                            self.failure = self.failure + 1;
                        end
                        if self.find_x(1) < 0
                            x_ref = -x_ref;
                        end
                        self.find_u = self.gain * (x_ref-self.find_x);
                    elseif strcmp(self.type, 'none')
                        self.find_u = zeros(3,1);
                    elseif strcmp(self.type, 'MPC')
                        [nx, nu] = size(B); 
                        np = 10;
                        try
                            Pf = dlyap(A+B*self.gain, self.Q+self.gain'*self.R*self.gain); %final state penalization
                            M = zeros(np*nx,nx); % prediction matrices
                            N = zeros(np*nx,nu*np); %size(N)
                            for i = 1 : np
                                M(nx*i-nx+1:nx*i,:) = A^i;
                                N1(nx*i-nx+1:nx*i,:) = A^(i-1)*B;
                            end
                            for i = 1 : np
                                N(nx*i-nx+1:end,nu*i-nu+1:nu*i) = N1(1:np*nx-(i-1)*nx,:);
                            end
                            H = zeros(np*nu,np*nu); %Hessian
                            G = zeros(np*nu,nx); %gradient
                            R1 = 0.05e4*eye(np*nu);
                            for i = 1 : np-1
                                H = H + N(nx*(i)-nx+1:nx*(i),:)'*self.Q*N(nx*(i)-nx+1:nx*(i),:);
                                G = G + N(nx*i-nx+1:nx*i,:)'*self.Q*M(nx*(i)-nx+1:nx*(i),:);
                            end
                            H1 = N(nx*np-nx+1:nx*np,:)'*Pf*N(nx*np-nx+1:nx*np,:)+R1;
                            G1 = N(nx*np-nx+1:nx*np,:)'*Pf*M((np)*nx-nx+1:(np)*nx,:);
                            H = H+H1;
                            G = G + G1;
                        catch
                            disp('error solving Lyapunov equation');
                            self.failure = self.failure + 10000;
                        end
                        lb = -self.bot_sat*ones(30,1);%self.bot_sat
                        ub = self.top_sat*ones(30,1);
                        lbA = ones(30,1)*-inf;%-self.bot_sat-M*self.find_x;
                        ubA = ones(30,1)*inf;%self.top_sat-M*self.find_x;


                        [x,fval,exitflag,iter,lambda,auxOutput] = qpOASES( H,G*(x_ref-self.find_x),eye(30),lb,ub,lbA,ubA);%lbA*ones(1,7),ubA*ones(1,7)
                        u_opt = x(1:3);
                        self.find_u = u_opt;
                    end
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
%                 self.failure = self.failure + 1;
%             end
        end
        
        function find_u = get_action_control(self)
            find_u = self.find_u;
        end

    end
end
