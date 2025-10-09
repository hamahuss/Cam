Inputs: pixel (u,v), camera intrinsics (u0,v0,fx,fy),
        camera-to-body rotation R_b_c,
        desired yaw psi_d,
        current attitude q, current yaw psi_meas,
        gains: lambda_ibvs, k_psi, Kp_att, Kd_att

1. Compute camera LOS:
   v_c = [ (u-u0)/fx, (v-v0)/fy, 1 ]'
   v_c_hat = v_c / norm(v_c)

2. Transform to body:
   v_b_hat = R_b_c * v_c_hat   # direction vector in body axes

3. Build desired attitude (recommended):
   v_tilde = R_z(-psi_d) * v_b_hat
   a = cross([0,0,1], v_tilde)
   if norm(a) < eps:
       q_tilt = identity_quat()
   else:
       axis = a / norm(a)
       angle = atan2(norm(a), dot([0,0,1], v_tilde))
       q_tilt = quaternion_from_axis_angle(axis, angle)
   q_yaw = quaternion_from_yaw(psi_d)
   q_des = quaternion_multiply(q_yaw, q_tilt)

4. Attitude error:
   q_e = quat_mul(quat_conj(q), q_des)
   e_att = q_e.vec * sign(q_e.w)   # 3-vector error
