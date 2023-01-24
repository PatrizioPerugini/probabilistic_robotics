1;
function delta = predict(x,u)
#x is the state -> ksteer ktraction steer_offset baseline
ksteer          = x(1);
ktraction       = x(2);
steer_offset    = x(3);
baseline        = x(4);
#u are the values that come from the encoders (delta values)
steer_ticks     = u(1);
traction_ticks  = u(2);
u1_k = k_traction*traction_ticks;
u2_k = k_steer*steer_ticks - steer_offset; #Not sure at all about this one 
#how will I define sin and cos ? -> which theta should I take into account... and which phi
c   = cos(theta);
s   = sin(theta);
tg  = tan(phi);
dx     = u1_k*c;
dy     = u1_k*s;
dtheta = u1_k*tg/l;
dphi   = u2_k;
delta = [dx, dy, dtheta, dphi];
#kinematic model
#x_dot = cos_theta*u1 
#y_dot = sin_theta*u1
#theta_dot = tg(phi)/l*u1
#phi_dot = u2

endfunction

function [e,J] = errorAndJacobian(x,u,z)

