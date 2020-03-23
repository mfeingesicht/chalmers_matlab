% Define the time vector for the simulation
time_array = 0:Ts:10*sim_time;
length1 = length(time_array);
length2 = floor(length1/10);

switch path    
    case 1
        % Straight Path ===================================================
        path_x = v*time_array;
        path_y = 0*path_x; 
    
    case 2 
        % Circle ==========================================================
        path_x = radius * sin(time_array * v / radius);
        path_y = radius * cos(time_array * v / radius ) - radius;
        
    case 3
        % _/-\_ ===========================================================
        path_x = v*time_array;
        path_y = 0*path_x;
        path_y(floor(length2/5)+1:floor(2*length2/5)) = slope*Ts*(0:(floor(length2/5)-1));
        path_y(floor(2*length2/5)+1:floor(3*length2/5)) = path_y(floor(2*length2/5));
        path_y(floor(3*length2/5)+1:floor(4*length2/5)) = path_y(floor(3*length2/5)) - slope*Ts*(0:(floor(length2/5)-1));
        
    case 4
        % Sinusoidal Path =================================================
        path_x = v*time_array;
%         path_y = sin(path_x*1);
        path_y = sin(path_x/20);
        
    case 5
        % _/-\_ integrated from heading ===================================
        heading_path = 0*time_array;
        heading_path(floor(length2/5)+1:floor(2*length2/5)) = atan(slope);
        heading_path(floor(3*length2/5)+1:floor(4*length2/5)) = -atan(slope);
        
        path_x = cumtrapz(time_array,v*cos(heading_path));
        path_y = cumtrapz(time_array,v*sin(heading_path));
end

path_x = path_x';
path_y = path_y';