1;
source "utils.m"


function delta = predict(x,u_a,u_i,dt=1)

    #kinematic model, we consider the model of a bicicletta:
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

    # "TRUE" PARAMETERS, USE ONLY FOR TESTS
                                           
    ksteer =               0.564107;         # 0.1
    ktraction=             0.0106141;        # 0.0106141
    baseline=              1.54757;          # 1.4
    steer_offset =        -0.0559079;        # 0

    #u are the values that come from the encoders (delta values)
    #u_a is the value coming from the absolute encoders
    #u_i is the value coming from the incremental encoders
    steer_ticks     = u_a;
    traction_ticks  = u_i;

    max_steer_ticks = 8192;
    max_traction_ticks = 5000;
    #get linear part
    
    u1_k = (ktraction*traction_ticks)/max_traction_ticks;
    #u1_k = (ktraction)*(traction_ticks/max_traction_ticks);
    
    #nomalize angle and retrieve angular part
    delta_a = (steer_ticks / max_steer_ticks)*(2*pi);
    u2_norm = atan2(sin(delta_a),cos(delta_a));
    u2_k = u2_norm *ksteer + steer_offset; 
   
    sampling_time =1;
    dphi   = u2_k;
    tg  = tan(dphi);
    dtheta = (u1_k*tg/baseline);

    
    c = cos(dtheta);
    s = sin(dtheta);

    dx     = u1_k*c;
    dy     = u1_k*s;

    delta = [dx, dy, dtheta]';
endfunction

function delta_noise = predict_noise(x,u_a,u_i,dt=1)
    x(1)+=0.05;
    x(2)+=0.01;
    x(3)+=0.1;
    delta_noise = predict(x,u_a,u_i,dt);
endfunction

#concatenate the incremental values obtained by the non calibrated parameters
function T = compute_odometry_trajectory(x,u_a,u_i,dt,b2s_v = [1.5,0,0])
   
    T = zeros(size(u_a,1),3);
    in_sensor = v2t(b2s_v);
    current_T = v2t(zeros(1,3));
    for i=1:size(u_a,1)
        #delta = GetIncrements(x,u_a(i),u_i(i),dt(i),i);
        delta = predict(x,u_a(i),u_i(i),dt(i));
        
        #current_T*=v2t(delta);
        current_T*=v2t(in_sensor*delta);
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

#function J = JPredict(x,u_a,u_i)
#    state_dim = 4;
#    J = zeros(3,4);
#    dx = zeros(state_dim,1);
#    global epsilon = 1e-4;
#    #apply finite differences
#    for i = 1:size(x,1)
#        dx(i) = epsilon;
#        J(:,i)=predict_noise(x+dx,u_a,u_i) - predict_noise(x-dx,u_a,u_i);
#        dx(i)=0;
#    endfor
#    J*=.5/epsilon;
#endfunction

#function [e,J] = errorAndJacobian(x,u_a,u_i,z)
#    
##    e = predict_noise(x,u_a,u_i) - z;
##    J = JPredict(x,u_a,u_i);#

#endfunction 

#function x_new = calibrate(x,U_a,U_i,z)
#    dim = 4;
#    x_new = x
#    n_iters = 2;
#    n_points = size(U_a,1)
#    for it=1:n_iters
#        H = zeros(dim,dim);
#        b = zeros(dim,1);
#        sum_chi = 0;
#        for i=1:n_points
#            #obs = z(i,:);
#            u_a = U_a(i);
#            u_i = U_i(i); 
#            obs = predict(x,u_a,u_i); 
#            [e, J] = errorAndJacobian(x_new,u_a,u_i,obs);
#            
#            H+=J'*J; 
#            b+=J'*e;
#            sum_chi+=e'*e;
#        endfor
#        disp("the value of the chi is: ");
#        it
#        sum_chi
#        dx=-H\b;
#        x_new = x+dx;
#    endfor
#endfunction

# the problem to solve now has the following parameters
# x = (k_steer, k_traction, baseline, steering_offset, x_b2s, y_b2s, theta_b2s) -> dim 7
# z = x_s, y_s, z_s
# J = 3 x 7 

function x_new = calibrate(initial_guess,U_a,U_i,z)
    
    #let's try if it works if the initial guess is the identity... 
    #otherwise I'll try with the linear relaxation
    in_tx = 1.5;
    in_ty = 0;
    in_th = 0;
    #quando funzionera farli parametrici...
    ksteer =               0.564107;         # 0.1
    ktraction=             0.0106141;        # 0.0106141
    baseline=              1.54757;          # 1.4
    steer_offset =        -0.0559079; 
    #x_new = [initial_guess(1),initial_guess(2),initial_guess(3),initial_guess(4),in_tx,in_ty,in_th]'
    threshold = 10;
    x_new = [0.564107,0.0106141,1.54757,-0.0559079,in_tx,in_ty,in_th]'
    #s2r = eye(3);
    s2r = [ 1 0 1.5;
            0 1 0;
            0 0 1];
    dim =7;
    num_iterations = 100;
    n_points = size(z,1)
    for it= 1:num_iterations
        H = zeros(dim,dim); b = zeros(dim,1);
        sum_chi = 0;
        for i = 1:n_points
            
            u_a = U_a(i);
            u_i = U_i(i); 
            #pose_r = predict(x,u_a(i),u_i(i),dt(i));#z_hat but in the baseline
            
            obs = z(i,:); #z in sensor frame
            #[e J] = errorAndJacobian_(s2r,pose_r,obs);
            [e, J] = errorAndJacobian_(x_new,u_a,u_i,obs);
            chi=e'*e;
            if chi > threshold
                e*=sqrt(threshold/chi);
                chi=threshold;
            endif
            H+=J'*J;
            b+=J'*e;
            
            sum_chi+= chi;
        endfor
        disp("the value of the chi is: ");
        it
        sum_chi
        #H
        #b
        dx = -H\b;
        #qua dx sarebbe un box plus per le ultie 3 componenti e un semplice piu per le prime 4
        x_new = boxplus_(x_new,dx)
    endfor    
endfunction

function J = JPredict(x,u_a,u_i,z_hat)
   x_t = [x(5);
           x(6);
           x(7)];
    #b2s = v2t(x_t);
    #b2s
    state_dim = 7;
    dx_k0_dim = 4;
    J = zeros(3,state_dim);
    #read the parameters of the baseline to do numerical diff
    k_x = x(1:4);
    dx = zeros(dx_k0_dim,1);
    global epsilon = 1e-4;
    #apply finite differences for firs 4 parameters
    for i = 1:size(k_x,1)
        dx(i) = epsilon;
        #if i == 2
        #  disp("-----") 
        #  disp( predict(k_x+dx,u_a,u_i)- predict(k_x-dx,u_a,u_i) )
        #  disp("-----") 
        #  disp( .5*(predict(k_x+dx,u_a,u_i)- predict(k_x-dx,u_a,u_i))/epsilon )
        #
        #endif
        J(:,i)= predict(k_x+dx,u_a,u_i) - predict(k_x-dx,u_a,u_i);
        dx(i)=0;
    endfor
    
    J*=.5/epsilon;
    #now fill the part relative to the pose pose
    J(1:2,5:6)=eye(2);
    J(1:2,7)=[-z_hat(2),
	  z_hat(1)]';
    
    
     
    
    
endfunction

function [e,J] = errorAndJacobian_(x,u_a,u_i,z)
    #boxminus.. or just normalize the angle penso vada bene
    x_t = [x(5);
           x(6);
           x(7)];
           
    b2s = v2t(x_t);
    z_hat = b2s*predict(x,u_a,u_i);
    
    #z_hat = v2t(predict(x,u_a,u_i))*x_t;
    
    #e = z'-z';
    e =  z_hat- z';
    e(3) = atan2(sin(e(3)),cos(e(3)));
    J = JPredict(x,u_a,u_i,z_hat);
    
   

endfunction


function [e J] = errorAndJacobian(X,P,Z)
    #P are the poses in baseline frame
    #Z are the poses in sensor frame
    #J is of dimension -> (error_dim,state_dim) 
    J = zeros(3,7); 

    #NOT 100% SURE ABOUT IT
    z_hat = v2t(X*P);

    #Xr = v2t(P);
    Zs = v2t(Z);
    #P
    #Xr
    
    #apply flattening 
    #ix_r=reshape(Xr(1:2,:),[],1)
    #zat=reshape(z_hat(1:2,:),[],1)
    #obd=reshape(Zs(1:2,:),[],1)
    e = reshape(z_hat(1:2,:),[],1) - reshape(Zs(1:2,:),[],1); #error in now a 1x6
    
    R = Xr(1:2,1:2);
    t = Xr(1:2,3);
    g_r = X(1:2,1:2);#actual guess for rotation
    g_t = X(1:2,3);#actual guess for traslation
    dr_a = [0 -1; #derivative of rotation evaluated in zero 
            1 0];
    j_dr = reshape(R*dr_a*g_r,[],1);
    j_t = R; #comes from -> R(R(da)t + dt)+t -> R(derive wrt t)
    #dx_t = 
    J=zeros(6,3);
    J(1:4,3) = j_dr;
    J(5:6,1:2)=j_t;
    #J(5:6,3) = R*dr_a*g_t;
    J(5:6,3) = R*g_t;
    
endfunction

function X = boxplus_(X,dx)
    X_transf = [X(5);
                X(6);
                X(7)];
    dx_s = [dx(5);
            dx(6);
            dx(7)];
    for i = 1:4
      X(i)+=dx(i);
    endfor
    
    #potrebbe esse sbagliata pe broadcasting
    #X(5:7) = v2t(X_transf)*dx_s;
    X(5:7) = v2t(dx_s)*X_transf;
endfunction
