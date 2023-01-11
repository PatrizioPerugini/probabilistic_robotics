1;
source "reader.m"
source "calibrate.m"
#addpath "calibrate.m"
filepath = "/dataset.txt";

#read the data

[t steer_t traction_t poses sensor_poses] = reader();

#calculate the transformation between the sensor and the baseline of the robot 
s2r = calibrate(poses,sensor_poses);
#s2r

function plot_trajectories(poses)
    figure();
    hold on;
    title("Trajectories comparison");
    plot(reshape(poses'(:, 1), 1, []), reshape(poses'(:, 2), 1, []), 'r-', 'linewidth', 3);   
    legend("true odometry")
endfunction

#plot_trajectories(poses)
