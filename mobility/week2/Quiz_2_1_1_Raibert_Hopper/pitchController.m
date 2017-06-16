function Tphi = pitchController(phi,phiDesired,dphi_dt)
kd_phi = 5;
kp_phi = 5;
Tphi= -1*(kp_phi*(phi-phiDesired) + kd_phi*dphi_dt);