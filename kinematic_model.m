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


function delta = predict(x,u_a,u_i,max_traction_ticks=5000,max_steer_ticks=8192)

    #kinematic model, we consider the model of a bicicletta:
        #x_dot = u1*cos_theta*cos(phi) 
        #y_dot = u1*sin_theta*cos(phi)
        #theta_dot = u1*sin(phi)/bl
        #phi_dot = u2

    ksteer          = x(1);
    ktraction       = x(2);
    baseline        = x(3);
    steer_offset    = x(4);


    #u are the values that come from the encoders (delta values)
    #u_a is the value coming from the absolute encoders
    #u_i is the value coming from the incremental encoders
    steer_ticks     = u_a;
    traction_ticks  = u_i;

    #get linear part
    
    u1_k = ktraction*(traction_ticks/max_traction_ticks);
    #get angular part
    if steer_ticks>max_steer_ticks/2
        steer_ticks-=max_steer_ticks;
    endif
    steer = steer_ticks/max_steer_ticks;
    u2_k = 2*pi*ksteer*steer+steer_offset; 
   
    phi  = u2_k;
    droh = u1_k;
    
    sin_phi = sin(phi);
    cos_phi = cos(phi);
    droh_base = droh*cos_phi;
    dtheta = droh*sin_phi/baseline;

    [c,s]   = thetaTerms(dtheta);

    dx     = droh_base*c;
    dy     = droh_base*s;
 
    delta = [dx, dy, dtheta]';
endfunction

function T = compute_true(x)
    T = zeros(size(x,1),3);
    current_T = v2t(zeros(1,3));
    for i=1:size(x,1)
      delta = x(i,1:3)';
      current_T*=v2t(delta);
      T(i,1:3)=t2v(current_T)';
    endfor
endfunction

#concatenate the incremental values obtained by the non calibrated parameters
function T = compute_odometry_trajectory_u(x,u_a,u_i,b2s_v = [1.5,0.0,0])
    T = zeros(size(u_a,1),3);
    in_sensor = v2t(b2s_v);
    current_T = v2t(zeros(1,3));
    for i=1:size(u_a,1)
        delta = predict(x,u_a(i),u_i(i));
        #CHANGE COMMENT FOR FINAL PROBLEM, THIS IS JUST A TEST
        current_T*=v2t(delta);
    
        T(i,1:3)=t2v(current_T)';
    end
endfunction

function T = compute_odometry_trajectory(x,u_a,u_i,b2s_v = [1.5,0.0,0.0])
  
    delta_sensor = zeros(size(u_a,1),3);
    Xs = v2t(b2s_v);
    current_T = v2t(zeros(1,3));
    for i=1:size(u_a,1)
        #delta = GetIncrements(x,u_a(i),u_i(i),dt(i),i);
        delta_r = predict(x,u_a(i),u_i(i));
        
        #CHANGE COMMENT FOR FINAL PROBLEM, THIS IS JUST A TEST
        delta_s = Xs^-1*v2t(delta_r)*Xs;
        #current_T*=v2t(delta);

        delta_sensor(i,1:3)=t2v(delta_s)';
    end
    T = compute_true(delta_sensor);
endfunction

# the problem to solve now has the following parameters
# x = (k_steer, k_traction, baseline, steering_offset, x_b2s, y_b2s, theta_b2s) -> dim 7
# z = x_s, y_s, z_s
# J = 3 x 7 

function [x_new, chis] = calibrate(initial_guess,U_a,U_i,z,ktm,ksm)
    in_tx = 1.5;
    in_ty = 0;
    in_th = 0;
    
    x_new = [initial_guess(1),initial_guess(2),initial_guess(3),initial_guess(4),in_tx,in_ty,in_th]'
    threshold = 1;
    dim =7;
    num_iterations = 70;
    n_points = size(z,1)
    #n_points = 200
    chis=[];
    for it= 1:num_iterations
        H = zeros(dim,dim); b = zeros(dim,1);
        sum_chi = 0;
       
        for i = 1:n_points
            u_a = U_a(i);
            u_i = U_i(i); 
            obs = z(i,:); 
            [e J] = erroeAndJacobians_fd(x_new,u_a,u_i,obs,ktm,ksm); 
            chi=e'*e;
            
            if chi > threshold
                e*=sqrt(threshold/chi);
                chi=threshold;
            endif

            omega = eye(3)*17;
            omega(2,2)*=0.3;
            H+=J'*omega*J;
            b+=J'*omega*e;
            #H+=J'*J;
            #b+=J'*e;
            
            sum_chi+= chi;
        endfor
       
        chis = [chis; sum_chi];
        it
        damp_H = H + eye(dim);
        dx = -damp_H\b;
      
        x_new = boxplus_(x_new,dx)
    endfor    
endfunction


function Jp = JPredictParams(x,u_a,u_i,Z,ktm,ksm)
   
    x_k = [x(1); x(2); x(3); x(4)];
    x_t = [x(5); x(6); x(7)];
    meas_dim = 3;
    state_dim = 7;
    k_dim = 4;
    Xs = v2t(x_t);
    Jp = zeros(meas_dim,state_dim);
    dx=zeros(k_dim,1);
    global epsilon=1e-9;
    for (i=1:k_dim)
      dx(i)=epsilon;
      d_plus  = t2v(Z^-1*Xs^-1*v2t(predict(x_k+dx,u_a,u_i,ktm,ksm))*Xs);
      d_minus = t2v(Z^-1*Xs^-1*v2t(predict(x_k-dx,u_a,u_i,ktm,ksm))*Xs);
      Jp(:,i)= d_plus- d_minus;
      dx(i)=0;
    endfor;
   
    Jp*=.5/epsilon;

endfunction

function Js = JPredictSensor(x,u_a,u_i,Z,ktm,ksm)
    
    x_k = [x(1); x(2); x(3); x(4)];
    x_t = [x(5); x(6); x(7)];
    meas_dim = 3;
    state_dim = 7;
    k_dim = 3;
    Js = zeros(meas_dim,state_dim);
    Xs = v2t(x_t);
    dx=zeros(k_dim,1);
    global epsilon=1e-9;
    #e = t2v(Z^-1* Xs^-1*v2t(Predict)*Xs)
    for (i=1:k_dim)
      dx(i)=epsilon;
      Xs = v2t(x_t);
      Xp = Xs*v2t(dx);
      d_plus  = t2v(Z^-1*Xp^-1*v2t(predict(x_k,u_a,u_i,ktm,ksm))*Xp);
      Xm = Xp = Xs*v2t(-dx);
      d_minus = t2v(Z^-1*Xm^-1*v2t(predict(x_k,u_a,u_i,ktm,ksm))*Xm);
      Js(:,4+i)= d_plus- d_minus;
      dx(i)=0;
    endfor;
    Js*=.5/epsilon;

endfunction 

function [e J] = erroeAndJacobians_fd(x,u_a,u_i,obs,ktm,ksm)
    
    #parameters of the mobile base
    x_k = [x(1); x(2); x(3); x(4)];
    #parameters of extrinsic cam
    x_t = [x(5); x(6); x(7)];
   
    Z = v2t(obs);
    #base of robot
    Xi = v2t(predict(x_k,u_a,u_i,ktm,ksm));
    #transform robot-sensor
    Xs = v2t(x_t);
    #prediction
    h_x = Xs^-1*Xi*Xs;
    #error function
    e = t2v(Z^-1*h_x);
    #Jacobian wrt kinematic parameters
    Jp = JPredictParams(x,u_a,u_i,Z,ktm,ksm);
    #Jacobian wrt T cam
    Js = JPredictSensor(x,u_a,u_i,Z,ktm,ksm);
   
    J=Jp+Js;
    
endfunction

function X = boxplus_(X,dx)
    X_s = [X(5);
                X(6);
                X(7)];

    dx_s = [dx(5);
            dx(6);
            dx(7)];
    for i = 1:4
      X(i)+=dx(i);
    endfor
  
    #X(5:7) = v2t(dx_s)*X_transf;
    X(5:7) = t2v(v2t(dx_s)*v2t(X_s));

endfunction
