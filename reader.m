#"dataset.txt"
#function [Time_stamp Steer_ticks Traction_tick Model_pose Tracker_pose ] = load_ds(filepath)
1;
source "utils.m"
function [times steer_ticks traction_ticks poses sensor_poses delta_sensor_poses initial_guess] = reader()
    #line3 is of interest since there is the initial guess
    #line5 is of interest since there are the max ranges
    filepath = "dataset.txt";
    poses = [];
    sensor_poses = [];
    delta_sensor_poses=[];
    times = [];
    steer_ticks = [];
    traction_ticks = [];
    k_steer = 0; k_traction = 0; axis_length = 0; steer_offset = 0; k_steer_max = 0; k_traction_max = 0;
    past_time=0; past_steer_ticks=0; past_traction_ticks=0;past_sensor_pose=[0 0 0];
    new_time=0; new_steer_ticks=0; new_traction_ticks=0;new_sensor_pose=[0 0 0];
    fid = fopen(filepath,'r');
    i = 1;
    while true
        line = fgetl(fid);
        if line == -1 #|| i == 35
            break
        end
        elems = strsplit(line,' ');
        if i == 3
            k_steer      =  str2double(elems{2});
            k_traction   =  str2double(elems{3});
            axis_length  =  str2double(elems{4});
            steer_offset =  str2double(elems{5});
        end
        if i ==5
            k_steer_max    = str2double(elems{2});
            k_traction_max = str2double(elems{3});
        end
        if i == 7
            t_guess = [elems{2}; elems{3} ;elems{4}];
        end
        if i == 8
            r_guess = [elems{3}; elems{4}; elems{5}; elems{6}];
        end
        if i ==9
            past_time=read_time(elems);
            past_steer_ticks=read_steer_ticks(elems); 
            past_traction_ticks=read_traction_ticks(elems);
            past_sensor_pose = read_sensor_pose(elems);
            
            
        end
        
        if i>9  #line 34 possibile errore... passiamo da -2 a 9 ....
            
            #TIME
            new_time = read_time(elems);
            delta_t = new_time-past_time;
            past_time=new_time;
            
            #STEER          
            new_steer_ticks    = read_steer_ticks(elems);

            #TRACTION
            
            new_traction_ticks = read_traction_ticks(elems);

            #pose

            new_sensor_pose = read_sensor_pose(elems);
            #if i==68
            #new_traction_ticks
            #past_traction_ticks
            delta_traction = new_traction_ticks-past_traction_ticks;

            #delta_sensor_pose = v2t(new_sensor_pose)-past_sensor_pose;
            delta_sensor_pose = t2v(v2t(past_sensor_pose)^-1*v2t(new_sensor_pose))';
            #disp("-----------")
            #i
            #delta_sensor_pose(3)=normalizeAngle(new_sensor_pose(3))
            #delta_sensor_pose
            past_sensor_pose = new_sensor_pose;


            if delta_traction < -100000
                max_int = 4294967295; # max value for uint32
                delta_traction = (max_int - past_traction_ticks) + new_traction_ticks;
            endif
            #con < e > di 5000 vie na cosa decente
            if delta_traction>5000
                
                #WARNINGGGGGGGGGG

                #disp("ops")
                #delta_t
                #delta_traction
                #disp("-----------")
                delta_traction=5000;
            endif
            if delta_traction < -5000
                delta_traction=-5000;
            endif
            past_traction_ticks=new_traction_ticks;
           
            #delta_traction
            #disp("-------------")
            
            #POSE
            new_pose = read_pose(elems);
            
            #SENSOR POSE
            #new_sensor_pose = read_sensor_pose(elems);
            
            #populate the structures 
            steer_ticks = [steer_ticks; new_steer_ticks];
            
            traction_ticks = [traction_ticks; delta_traction];
            poses =[poses; new_pose] ;
            
            sensor_poses = [sensor_poses;new_sensor_pose];
            delta_sensor_poses = [delta_sensor_poses;delta_sensor_pose];
            
            times = [times;delta_t];

        end

    
        
        i+=1;
   
    end
    #initialize initial guess
    initial_guess = [k_steer; k_traction; axis_length; steer_offset];# t_guess; r_guess];
    
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





