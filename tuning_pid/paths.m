% Define the time vector for the simulation
step_time = Ts;
total_time = sim_time*5;
time_array = 0:step_time:total_time;
length1 = size(time_array,2);


% Initial position for the path
x_ini_diff = 0;
y_ini_diff = 0;


switch path
    
 case 1
        %Straight Path -- =================================================
        path_x = time_array;
        path_y = 0*path_x;
        initial_x = x_ini_diff;
        initial_y = y_ini_diff;   
    
    case 2 
        %Circle ===========================================================
        path_x = radius * sin(2 * time_array / 10);
        path_y = radius * cos(2 * time_array / 10) - radius;
        initial_y = y_ini_diff;
        initial_x = y_ini_diff;
        
    case 3
        %_/- ==============================================================
        path_x = time_array;
        path_y = 0*path_x;
        path_y(round(length1/3):round(2*length1/3)) = slope*path_x(1:round(length1/3));
        path_y(round(2*length1/3)+1:end) = path_y(round(2*length1/3));
        initial_x = x_ini_diff;
        initial_y = y_ini_diff;  
    
    case 4
        %___O___ ==========================================================
        path_x1 = linspace(0,2*radius,100);
        path_y1 = 0*path_x1;
        path_x3 = linspace(2*radius,4*radius,100);
        path_x3(1) = [];
        path_y3 = 0*path_x3;
        
        angle2 = linspace(0,2*pi,100);
        path_x2 = radius * sin(angle2)+2*radius;
        path_x2(1) = [];
        path_y2 = radius * cos(angle2)-radius;     
        path_y2(1) = [];
        path_x = [path_x1 path_x2 path_x3];
        path_y = [path_y1 path_y2 path_y3];
        initial_y = y_ini_diff;
        initial_x = x_ini_diff; 

    case 5
        % Straight Path with Oscillations 1 / =============================
        path_x = time_array;
        path_y = slope * path_x + sin(path_x*.1); % INTERESTING
        initial_x = x_ini_diff;
        initial_y = y_ini_diff; 

    case 6
        % Straight Path with Oscillations 2 / =============================
        path_x = time_array;
        path_y = slope * (path_x + 6 * sin(path_x*0.20)); % BAD
        initial_x = x_ini_diff;
        initial_y = y_ini_diff; 
        
    case 7
        % Straight Path / =================================================
        path_x = time_array;
        path_y = 0.2*path_x; % BAD
        initial_x = x_ini_diff;
        initial_y = y_ini_diff;         
        
    case 8
        % Sinusoidal Path =================================================
        path_x = time_array;
        path_y = 4 * sin((path_x*2*pi)/100); % GOOD
        initial_x = x_ini_diff;
        initial_y = y_ini_diff; 

    case 9
        %Increasing Oscillating Sinusoidal Path ===========================
        path_x = time_array;
        path_y = sqrt(path_x * 0.8) .* sin(0.1*path_x); % GOOD
        initial_x = x_ini_diff;
        initial_y = y_ini_diff; 
     case 10
        %___/ 45º curve ==========================================================
        path_x1 = linspace(0,3*radius,100);
        path_y1 = 0*path_x1;
        path_x3 = linspace(3*radius+0.5*sqrt(2)*radius,6*radius,100);
        path_x3(1) = [];
        path_y3 = linspace((-radius+0.5*sqrt(2)*radius),-4*radius,100);
        path_y3(1) = [];
        
        angle2 = linspace(0,0.25*pi,100);
        path_x2 = radius * sin(angle2)+3*radius;
        path_x2(1) = [];
        path_y2 = radius * cos(angle2)-radius;
        path_y2(1) = [];
        path_x = [path_x1 path_x2 path_x3];
        path_y = [path_y1 path_y2 path_y3];
        initial_y = y_ini_diff;
        initial_x = x_ini_diff;
        
    case 11
        path_x1 = linspace(0,3*radius,100);
        path_y1 = 0*path_x1;        
        path_y3 = (linspace(-radius,-4*radius,100));
        path_y3(1) = [];
        path_x3 = (4*radius*ones(size(path_y3)));

        angle2 = linspace(0,0.5*pi,100);
        path_x2 = (radius * sin(angle2))+3*radius;
        path_x2(1) = [];
        path_y2 = (radius * cos(angle2))-radius;      
        path_y2(1) = [];
        path_x = [path_x1 path_x2 path_x3];
        path_y = [path_y1 path_y2 path_y3];
        initial_y = y_ini_diff;
        initial_x = x_ini_diff;
        
    case 12
        %way turn and back ==========================================================        

        path_x1 = linspace(0,2*radius,100);
        path_y1 = 0*path_x1;
        path_x3 = linspace(2*radius,-radius,100);
        path_x3(1) = [];
        path_y3 = -2*radius*ones(size(path_x3));
        
        angle2 = linspace(0,pi,100);
        path_x2 = radius * sin(angle2)+(2*radius);
        path_x2(1) = [];
        path_y2 = radius * cos(angle2)-radius;        
        path_y2(1) = [];
        path_x = [path_x1 path_x2 path_x3];
        path_y = [path_y1 path_y2 path_y3];
        initial_y = y_ini_diff;
        initial_x = x_ini_diff;
        
    case 14
        %overtaking ==========================================================        

        x1_in = x_ini_diff;
        x1_out = 2*radius;
        path_x1 = linspace(x1_in,x1_out,100);
        path_y1 = 0*path_x1;
        
        angle2 = linspace(-0.5*pi,-0.25*pi,100);
        path_x2 = radius * cos(angle2)+x1_out;
        path_x2(1) = [];
        path_y2 = radius * sin(angle2)+radius;
        path_y2(1) = [];
        
        x3_in = x1_out+0.5*sqrt(2)*radius;
        x3_out = x3_in+0.001*radius;
        path_x3 = linspace(x3_in,x3_out,100);
        path_x3(1) = [];
        y3_in = ((1-0.5*sqrt(2))*radius);
        y3_out = ((1-0.5*sqrt(2))*radius)+0.001*radius;
        path_y3 = linspace(y3_in,y3_out,100);
        path_y3(1) = [];
        
        angle4 = linspace(0.75*pi,0.5*pi,100);
        path_x4 = radius * cos(angle4)+x3_out+0.5*sqrt(2)*radius;
        path_x4(1) = [];
        path_y4 = radius * sin(angle4)+y3_out-0.5*sqrt(2)*radius;
        path_y4(1) = [];
        
        x5_in = x3_out+0.5*sqrt(2)*radius;
        x5_out = x5_in+1*radius;
        path_x5 = linspace(x5_in,x5_out,100);
        path_x5(1) = [];
        y5_in = y3_out+(1-0.5*sqrt(2))*radius;
        y5_out = y5_in;
        path_y5 = linspace(y5_in,y5_out,100);
        path_y5(1) = [];
        
        angle6 = linspace(0.5*pi,0.25*pi,100);
        path_x6 = radius * cos(angle6)+x5_out;
        path_x6(1) = [];
        path_y6 = radius * sin(angle6)+y5_out-radius;
        path_y6(1) = [];
        
        x7_in = x5_out+0.5*sqrt(2)*radius;
        x7_out = x7_in+0.001*radius;
        path_x7 = linspace(x7_in,x7_out,100);
        path_x7(1) = [];
        y7_in = ((1-0.5*sqrt(2))*radius)+0.001*radius;
        y7_out = ((1-0.5*sqrt(2))*radius);
        path_y7 = linspace(y7_in,y7_out,100);
        path_y7(1) = [];
        
        angle8 = linspace(-0.75*pi,-0.5*pi,100);
        path_x8 = radius * cos(angle8)+x7_out+0.5*sqrt(2)*radius;
        path_x8(1) = [];
        path_y8 = radius * sin(angle8)+y7_out+0.5*sqrt(2)*radius;
        path_y8(1) = [];
        
        x9_in = x7_out+0.5*sqrt(2)*radius;
        x9_out = x9_in+2*radius;
        path_x9 = linspace(x9_in,x9_out,100);
        path_x9(1) = [];
        path_y9 = 0*path_x9;
        
        path_x = [path_x1 path_x2 path_x3 path_x4 path_x5 path_x6 path_x7 path_x8 path_x9];
        path_y = [path_y1 path_y2 path_y3 path_y4 path_y5 path_y6 path_y7 path_y8 path_y9];
        initial_y = y_ini_diff;
        initial_x = x_ini_diff;
        
    case 15
        %_/- ==========================================================     
        
        x1_in = x_ini_diff;
        x1_out = 4*radius;
        path_x1 = linspace(x1_in,x1_out,100);
        path_y1 = 0*path_x1;
        
        angle2 = linspace(-0.5*pi,-0.25*pi,100);
        path_x2 = radius * cos(angle2)+x1_out;
        path_x2(1) = [];
        path_y2 = radius * sin(angle2)+radius;
        path_y2(1) = [];
        
        x3_in = x1_out+0.5*sqrt(2)*radius;
        x3_out = x3_in+0.5*radius;
        path_x3 = linspace(x3_in,x3_out,100);
        path_x3(1) = [];
        y3_in = ((1-0.5*sqrt(2))*radius);
        y3_out = ((1-0.5*sqrt(2))*radius)+0.5*radius;
        path_y3 = linspace(y3_in,y3_out,100);
        path_y3(1) = [];
        
        angle4 = linspace(0.75*pi,0.5*pi,100);
        path_x4 = radius * cos(angle4)+x3_out+0.5*sqrt(2)*radius;
        path_x4(1) = [];
        path_y4 = radius * sin(angle4)+y3_out-0.5*sqrt(2)*radius;
        path_y4(1) = [];
        
        x5_in = x3_out+0.5*sqrt(2)*radius;
        x5_out = x5_in+5*radius;
        path_x5 = linspace(x5_in,x5_out,100);
        path_x5(1) = [];
        y5_in = y3_out+(1-0.5*sqrt(2))*radius;
        y5_out = y5_in;
        path_y5 = linspace(y5_in,y5_out,100);
        path_y5(1) = [];
        
        path_x = [path_x1 path_x2 path_x3 path_x4 path_x5];
        path_y = [path_y1 path_y2 path_y3 path_y4 path_y5];
        initial_y = y_ini_diff;
        initial_x = x_ini_diff;        

end