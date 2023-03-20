1;
source "reader.m"
source "kinematic_model.m"
filepath = "/dataset_.txt";
#read the data
#It outputs 
#1 steer encoder ticks    -> absolute values
#2 traction encoder ticks -> delta values
#3 poses                  -> not needed
#4 sensor_poses           -> the true values coming from an external sensor
#  (delta sensor poses actually used)
#5 initial_guess          -> initial guess of the calibration parameters

[dt steer_t traction_t poses sensor_poses delta_sensor_poses initial_guess ktm ksm] = reader(false);

[x_new chis] = calibrate(initial_guess,steer_t,traction_t,delta_sensor_poses,ktm,ksm);
#after some iterations I got
#[0.5536, 0.0081,1.0796,-0.0015,1.379,-0.02,0.004]
x_new

x_k = x_new(1:4);
x_b2s = x_new(5:7);
h = figure(1)

#step 1 display the true trajectory
TrueTrajectory = sensor_poses;

disp("GT odometry");
hold on;

plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'bx', 'linewidth', 2);

#pause(1);

##step 2 display the trajectory with the uncalibrated parameters

Unc_odom_trajectory = compute_odometry_trajectory_u(initial_guess,steer_t,traction_t,dt);
#
disp("uncalibrated");
hold on;
plot(Unc_odom_trajectory(:,1),Unc_odom_trajectory(:,2), 'g-', 'linewidth', 2);

#pause(1);

##step 2 find the parameters of the odomotery... solve the least square problem
#
#final_odometry = compute_odometry_trajectory_noise(x_k,steer_t,traction_t,dt,x_b2s);
#
calibrated_odometry = compute_odometry_trajectory(x_k,steer_t,traction_t,x_b2s);
disp("final odometry");
hold on;
plot(calibrated_odometry(:,1),calibrated_odometry(:,2), 'rx', 'linewidth', 2);

#pause(1);



set( gca, 'fontsize', 16 );
legend( 'GT', 'IG', 'Calibrated' );
legStr = { 'GT', 'IG', 'Calibrated' };
legend( legStr, 'orientation', 'horizontal', ...
  'location', 'southoutside' );
waitfor(h);
