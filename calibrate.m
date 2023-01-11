
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
    s2r = eye(3);
    dim =3;
    num_iterations = 20;
    n_points = size(poses,1)
    for it= 1:num_iterations
        H = zeros(dim,dim); b = zeros(dim,1);
        chi = 0;
        for i = 1:n_points
            pose_r = poses(i,:);
            obs = sensor_poses(i,:);
            [e J] = errorAndJacobian(s2r,pose_r,obs);
        endfor
    #sensor poses are my observations 
    #poses*trasform_to_estimate is my prediction 
    #my error is poses*transform - sensor_poses
    endfor    
endfunction

function [e J] = errorAndJacobian(X,P,Z)
    
    #R = X(1:2,1:2);
    #T = X(1:2,3);
    #still not correct bc I need boxminus and boxplus (or to apply the flattening technique)
    z_hat = P*X;
    e = z_hat - Z;
    e
    J=0;
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