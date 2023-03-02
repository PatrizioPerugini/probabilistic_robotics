
#what shoul I do in this function:
#I have my true [x,y,theta] and through the ticks I have to create an estimate [x,y,theta]
#the error is then computed between these 2. my state in the first place is just 

#STATE:
#ksteer 
#ktraction 
#steer_offset 
#base_line 
1;
#s2r is the function which is used to calculate the the transformation between the sensor and the baseline 
#given the true position of the robot and of the sensor in the world
function s2r = calibrate(poses,sensor_poses)
    
    #let's try if it works if the initial guess is the identity... 
    #otherwise I'll try with the linear relaxation
    threshold = 1;
    deg = deg2rad(20);
    s2r = eye(3);
    #s2r = [cos(deg) -sin(deg) 0.1;
    #        sin(deg) cos(deg)  0.1;
    #        0 0 1]
    dim =3;
    num_iterations = 100;
    n_points = size(poses,1)
    for it= 1:num_iterations
        H = zeros(dim,dim); b = zeros(dim,1);
        sum_chi = 0;
        for i = 1:n_points
            pose_r = poses(i,:);
            obs = sensor_poses(i,:);
            [e J] = errorAndJacobian(s2r,pose_r,obs);
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
        sum_chi
        dx = -H\b;
        s2r = boxplus(s2r,dx);
        #t2v(s2r)
    #sensor poses are my observations 
    #poses*trasform_to_estimate is my prediction 
    #my error is poses*transform - sensor_poses
    endfor    
endfunction



function [e J] = errorAndJacobian(X,P,Z)
    
    Xr = v2t(P);
    Zs = v2t(Z);
    #P
    #Xr
    z_hat = Xr*X;
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

function s2r = boxplus(s2r,dx)
    s2r = v2t(dx)*s2r;
endfunction

function v = t2v(A)
	v(1:2, 1) = A(1:2, 3);
	v(3, 1) = atan2(A(2, 1), A(1, 1));
endfunction

function A = v2t(v)
  	c = cos(v(3));
  	s = sin(v(3));
	A = [c, -s, v(1);
         s, c, v(2);
         0, 0, 1];
endfunction