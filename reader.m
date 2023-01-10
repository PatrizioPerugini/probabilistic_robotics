#"dataset.txt"
#function [Time_stamp Steer_ticks Traction_tick Model_pose Tracker_pose ] = load_ds(filepath)
function [times steer_ticks traction_ticks poses sensor_poses] = reader()
    #line3 is of interest since there is the initial guess
    #line5 is of interest since there are the max ranges
    filepath = "dataset.txt";
    poses = [];
    sensor_poses = [];
    times = [];
    steer_ticks = [];
    traction_ticks = [];
    k_steer = 0; k_traction = 0; axis_length = 0; steer_offset = 0; k_steer_max = 0; k_traction_max = 0;
    fid = fopen(filepath,'r');
    i = 1;
    while true
        line = fgetl(fid);
        if line == -1 #|| i == 35
            break
        end
        elems = strsplit(line,' ');
        if i == 3
            k_steer      =elems{2};
            k_traction   =elems{3};
            axis_length  =elems{4};
            steer_offset =elems{5};
        end
        if i ==5
            k_steer_max = elems{2};
            k_traction_max = elems{3};
        end
        if i>6 #line 34 possibile errore... passiamo da -2 a 9 ....
            #Get new readings
            new_times = read_time(elems);
            new_steer_ticks    = read_steer_ticks(elems);
            new_traction_ticks = read_traction_ticks(elems);
            new_pose = read_pose(elems);
            new_sensor_pose = read_sensor_pose(elems);
            #populate the structures - encoders are not incremental...yet
            steer_ticks = [steer_ticks; new_steer_ticks];
            traction_ticks = [traction_ticks; new_traction_ticks];
            poses =[poses; new_pose] ;
            sensor_poses = [sensor_poses;new_sensor_pose];
            times = [times;new_times];
        end
        elems{1};
        i+=1;
   
    end
    #initialize initial guess
    initial_guess = [k_steer; k_traction; axis_length; steer_offset];
    poses;
end

function out = read_pose (elems)
    x_pose = str2double(elems{7});
    y_pose = str2double(elems{8});
    theta_pose = str2double(elems{9});
    out = [x_pose y_pose theta_pose];
end

function out = read_sensor_pose (elems)
    xs_pose = str2double(elems{11});
    ys_pose = str2double(elems{12});
    thetas_pose = str2double(elems{13});
    out = [xs_pose ys_pose thetas_pose];
end

function out = read_time (elems)
    t = str2double(elems{2});
    out = t;
end
function out = read_steer_ticks (elems)
    t = str2double(elems{4});
    out = t;
end
function out = read_traction_ticks (elems)
    t = str2double(elems{5});
    out = t;
end





