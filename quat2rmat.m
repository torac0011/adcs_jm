function m = quat2rmat(q)
    e1p = quat_rotate([1; 0; 0], q);
    e2p = quat_rotate([0; 1; 0], q);
    e3p = quat_rotate([0; 0; 1], q);
    m = [e1p, e2p, e3p];
end