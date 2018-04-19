function v_rot = quat_rotate(v, q)
    qv = [0; v];
    qv_rot = quat_mult(quat_mult(q, qv), quat_conj(q));
    v_rot = qv_rot(2:4);
end