1;
source "reader.m"
source "calibrate.m"
source "kinematic_model.m"
source "tests.m"
#addpath "calibrate.m"
filepath = "/dataset.txt";

#read the data
#It outputs 
#1 steer encoder ticks    -> absolute values
#2 traction encoder ticks -> delta values
#3 poses                  -> not needed
#4 sensor_poses           -> the true values coming from an external sensor
#5 initial_guess          -> initial guess of the calibration parameters
#6 guess_trans            -> initial guess of the traslation of the sensor wrt kinematic center TBD
#7 guess_rot              -> initial guess of orientation of the sensort wrt kinematic center TBD


[dt steer_t traction_t poses sensor_poses delta_sensor_poses initial_guess] = reader();

#x_new = calibrate(initial_guess,steer_t,traction_t,poses)# NOOOO
#traction_t
#x_new = initial_guess;

#x_new = calibrate_test(initial_guess,steer_t,traction_t,poses)

#x_new = calibrate(initial_guess,steer_t,traction_t,delta_sensor_poses)
#x_k = x_new(1:4);
#x_b2s = x_new(5:7);
h = figure(1)

#step 1 display the true trajectory

#TrueTrajectory = delta_sensor_poses;

TrueTrajectory = compute_odometry_trajectory(initial_guess,steer_t,traction_t);
disp("ground truth");
hold on;

#plot

plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);
pause(1);
#
##step 2 display the trajectory with the uncalibrated parameters
##To compute it let's try to insert the increments we have insiede the kinematic model 
##I chose and let's only use the uncalibrated parameters
#
#
#Odom_trajectory = compute_odometry_trajectory(initial_guess,steer_t,traction_t,dt);
#Odom_trajectory = compute_odometry_trajectory(x_new,steer_t,traction_t,dt);
Odom_trajectory = sensor_poses;
#
disp("uncalibrated");
hold on;
plot(Odom_trajectory(:,1),Odom_trajectory(:,2), 'g-', 'linewidth', 2);
pause(1);
#
##step 2 find the parameters of the odomotery... solve the least square problem
#
#final_odometry = compute_odometry_trajectory_noise(x_k,steer_t,traction_t,dt,x_b2s);
#

#disp("final odometry");
#hold on;
#plot(final_odometry(:,1),final_odometry(:,2), 'b-', 'linewidth', 2);
#pause(1);

#TBD

#step 3 use the calibrated parameters to get the calibrated odometry

waitfor(h);








#calculate the transformation between the sensor and the baseline of the robot 


#s2r = calibrate(poses,sensor_poses);
#s2r
#
function plot_trajectories(sensor_poses)
    figure();
    hold on;
    title("Trajectories comparison");
    x = sensor_poses(:,1);
    y = sensor_poses(:,2);
    plot(x, y, 'r-', 'linewidth', 2);
    pause(20);  
    legend("true odometry")
endfunction
#plot_trajectories(sensor_poses)
