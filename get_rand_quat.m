function q = get_rand_quat()
    sigma = rand() * 2*pi; % uniformly distributed [0 2pi)
    tau = rand() * 2*pi; % uniformly distributed [0 2pi)
    mu = rand(); % uniformly distributed [0 1]
    q = [sqrt(mu) * cos(sigma);
         sqrt(mu) * sin(sigma);
         sqrt(1 - mu) * sin(tau);
         sqrt(1 - mu) * cos(tau)];
end