1;
source "utils.m"

global sin_cos_coeffs = [1, 1/2, -1/6, -1/24, 1/120, 1/720];

function [s,c]=thetaTerms(x)
  global sin_cos_coeffs;
  a=1;
  s=0;
  c=0;
  for (i=1: size(sin_cos_coeffs,2))
    if (mod(i-1,2)==0)
      s+= a * sin_cos_coeffs(i);
    else
      c+= a * sin_cos_coeffs(i);
    endif;
    a*=x;
  endfor;
endfunction

function [s,c]=dThetaTerms(x)
  global sin_cos_coeffs;
  a=1;
  s=0;
  c=0;
  for (i=2: size(sin_cos_coeffs,2))
    if (mod(i-1,2)==0)
      s+= (i-1)*a * sin_cos_coeffs(i);
    else
      c+= (i-1)*a * sin_cos_coeffs(i);
    endif;
    a*=x;
  endfor;
endfunction

function delta = predict(x,u_a,u_i,dt=1)

    #kinematic model:
        #x_dot = cos_theta*u1 
        #y_dot = sin_theta*u1
        #theta_dot = u1*tg(phi)/l
        #phi_dot = u2

    #x is the state -> ksteer ktraction steer_offset baseline
    #     0.564107, 0.0106141, 1.54757, -0.0559079

    #ksteer          = x(1);
    #ktraction       = x(2);
    #baseline        = x(3);
    #steer_offset    = x(4);
    ksteer = 0.564107;
    ktraction=0.0106141;
    baseline=1.54757;
    steer_offset =-0.0559079;


    #u are the values that come from the encoders (delta values)
    steer_ticks     = u_a;
    traction_ticks  = u_i;

    max_ticks = 8192;
    u1_k = (ktraction*traction_ticks)/5000;#/dt
    #u1_k = (ktraction*mod(traction_ticks,5000))#/dt
    
    u2_k = ((steer_ticks / max_ticks)*(2*pi) - pi ) *ksteer + steer_offset; 
    #how will I define sin and cos ? -> which theta should I take into account... and which phi

    sampling_time =1;
    #sampling_time =0.001;
    dphi   = u2_k;
    tg  = tan(dphi);
    dtheta = (u1_k*tg/baseline)*sampling_time;#*(3.14/180);

    #[s,c]   = thetaTerms(dtheta);

    c = cos(dtheta);
    s = sin(dtheta);

    dx     = u1_k*c*sampling_time;
    dy     = u1_k*s*sampling_time;


    delta = [dx, dy, dtheta]';
    #delta = [dy, dx, dtheta]';
 

endfunction

function delta_noise = predict_noise(x,u_a,u_i,dt=1)
    x(1)+=0.05;
    x(2)+=0.01;
    x(3)+=0.1;
    delta_noise = predict(x,u_a,u_i,dt);
endfunction


#u_a stands for absolute 
#u_i stands for incremental

function delta = GetIncrements(x,u_a,u_i,dt,i)

    max_ticks = 8192;
    ksteer          = x(1); 
    ktraction       = x(2);
    steer_offset    = x(3);
    baseline        = x(4);
    baseline = 1.4;
    #u are the values that come from the encoders (delta values)
    steer_ticks     = u_a; 
    traction_ticks  = u_i;
    
    #obtain steereng angle from absolute encoders
    steering_angle = (u_a / max_ticks)*(2*pi)*ksteer + steer_offset;
    ang = steering_angle*(180/3.14);
    #obtain velocity from incremental encoders
    
    sampling_time = 0.01;
    v = (ktraction*u_i/dt)*sampling_time;
    if v > 30 || v <-30
        disp(v)
        disp(i)
        disp("-------")
    end
    dbeta_ = atan(tan(steering_angle)/baseline);

    dpsi_ = (v*cos(dbeta_)*tan(steering_angle)/baseline)*sampling_time;


    dx_ = v * cos(dpsi_ + dbeta_)*sampling_time;
  
    
    dy_ =v * sin(dpsi_ + dbeta_)*sampling_time;

    delta = [dx_; dy_; dpsi_] ;

endfunction

#concatenate the incremental values obtained by the non calibrated parameters
function T = compute_odometry_trajectory(x,u_a,u_i,dt)
   
    T = zeros(size(u_a,1),3);
    in_sensor = v2t([1.5 0 0])
    current_T = v2t(zeros(1,3));
    for i=1:size(u_a,1)
        #delta = GetIncrements(x,u_a(i),u_i(i),dt(i),i);
        delta = predict(x,u_a(i),u_i(i),dt(i));
        delta
        current_T*=v2t(delta);
        #current_T*=v2t(in_sensor*delta);
		T(i,1:3)=t2v(current_T)';
    end
endfunction

#this is just a test... the true values are now the values of the initial guess
function T = compute_odometry_trajectory_noise(x,u_a,u_i,dt)
   
    T = zeros(size(u_a,1),3);
    in_sensor = v2t([1.5 0 0])
    current_T = v2t(zeros(1,3));
    for i=1:size(u_a,1)
        #delta = GetIncrements(x,u_a(i),u_i(i),dt(i),i);
        delta = predict_noise(x,u_a(i),u_i(i),dt(i));
        current_T*=v2t(delta);
		T(i,1:3)=t2v(current_T)';
    end

    
endfunction

function J = JPredict(x,u_a,u_i)
    state_dim = 4;
    J = zeros(3,4);
    dx = zeros(state_dim,1);
    global epsilon = 1e-4;
    #apply finite differences
    for i = 1:size(x,1)
        dx(i) = epsilon;
        J(:,i)=predict_noise(x+dx,u_a,u_i) - predict_noise(x-dx,u_a,u_i);
        dx(i)=0;
    endfor
    J*=.5/epsilon;
endfunction

function [e,J] = errorAndJacobian(x,u_a,u_i,z)
    
    e = predict_noise(x,u_a,u_i) - z;
    J = JPredict(x,u_a,u_i);

endfunction 

function x_new = calibrate(x,U_a,U_i,z)
    dim = 4;
    x_new = x
    n_iters = 2;
    n_points = size(U_a,1)
    for it=1:n_iters
        H = zeros(dim,dim);
        b = zeros(dim,1);
        sum_chi = 0;
        for i=1:n_points
            #obs = z(i,:);
            u_a = U_a(i);
            u_i = U_i(i); 
            obs = predict(x,u_a,u_i); 
            [e, J] = errorAndJacobian(x_new,u_a,u_i,obs);
            
            H+=J'*J; 
            b+=J'*e;
            sum_chi+=e'*e;
        endfor
        disp("the value of the chi is: ");
        it
        sum_chi
        dx=-H\b;
        x_new = x+dx;
    endfor
endfunction
