clear all
syms w1 w2 w3 w_hat1 w_hat2 w_hat3 t1 t2 t3 B1 B2 B3 M1 M2 M3 I11 I22 I33 real
w = [w1; w2; w3];
w_hat = [w_hat1; w_hat2; w_hat3];
t = [t1; t2; t3];
B = [B1 B2 B3]';
M = [M1 M2 M3]';
I = [I11, 0, 0;
     0, I22, 0;
     0, 0, I33];
I_inv = inv(I);

syms q1 q2 q3 q4 a1 a2 a3 real

q_ref = [q1; q2; q3; q4];
a = [a1; a2; a3];

delta_q = 1/sqrt(4 + a'*a) * [2; a1; a2; a3];

dqr = delta_q(1);

dqv = delta_q(2:4);

delta_q_dot = 1/2 * quat_mult(delta_q, [0; w]) ...
              -1/2 * quat_mult([0; w_hat], delta_q);
          
% function rows          
a_dot = 2*(delta_q_dot(2:4)/delta_q(1) - delta_q(2:4)*delta_q_dot(1)/delta_q(1)^2);

w_dot = I_inv * (cross(M,B) - t - cross(w, I*w));

f = [a_dot; w_dot];

f_hat = subs(f, [t; a; w], [zeros(3,1); zeros(3,1); w_hat]);
% f_hat = subs(f, [t; a; w; B; M], [zeros(3,1); zeros(3,1); w_hat; rand(3,1)*1e-7; rand(3,1)]);
%f_hat = f_hat(4:6);

inertia = [1.92e3, 2.46e3, 2.34e3]/1e6;

f__ = matlabFunction(subs(f_hat, I, diag(inertia)), 'Vars', {w_hat, B, M});
% B = double(B);
% M = double(M);
f_anon = @(t, x_) f__(x_(4:6), rand(3,1)*1e-7, rand(3,1)) % parameters for ode45

x_state = [zeros(3,1); rand(3,1)*20/180*pi];
B_state = rand(3,1);
u_state = zeros(3,1);

% init_state = [x_state, B_state, u_state];
% f_anon(rand(6,1), B_state, u_state)
[t, x] = ode45(f_anon, [0 0.1], x_state);
