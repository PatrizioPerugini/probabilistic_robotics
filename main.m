source "reader.m"
source "calibrate.m"
filepath = "/dataset.txt";
[t steer_t traction_t poses sensor_poses] = reader();
steer_t
traction_t
#fplot(poses(:,1:3))