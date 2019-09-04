% Create a vector of velocities at which LQR gains will be computed
velocity_lqr = [input_velocity - 0.1 , input_velocity , input_velocity + 0.1];


%% Obtain LQR gains
% Initialize gains and eigenvalues matrices
lqr_gains = zeros(length(velocity_lqr),4);
closed_loop_eigenvalues = zeros(4,length(velocity_lqr));


% Check if fork dynamics are taken into account and compute LQR gains
if fork_dynamics_lqr==1
    
    for i = 1:length(velocity_lqr)
        v = velocity_lqr(i);

        A = [0  0   1   0;
             0  0   0   0;
             g/h_real    sin(lambda_real)*(h_real*v^2-g*a_real*c_real)/(h_real^2*b_real)   0    0;
             0  1   0   0];
        B = [0; 1; (a_real*v*sin(lambda_real))/(h_real*b_real); 0];
        sys_con = ss(A,B,C,D);
        sys_dis = c2d(sys_con,Ts);

        K = lqrd(A,B,Q,R,Ts);
        lqr_gains(i,:) = K;

        sysc = ss(A,B,C,D);
        sysd = c2d(sysc,Ts);
    end
    
else
    
    for i = 1:length(velocity_lqr)
        v = velocity_lqr(i);

        A = [0      0            1      0;
             0      0            0      0;
             g/h_real    v^2/(h_real*b_real)   0        0;
             0      1            0      0];
        B = [0; 1; a_real*v/(h_real*b_real); 0];
        sys_con = ss(A,B,C,D);
        sys_dis = c2d(sys_con,Ts);

        K = lqrd(A,B,Q,R,Ts);
        lqr_gains(i,:) = K;

        sysc = ss(A,B,C,D);
        sysd = c2d(sysc,Ts); 
    end
    
end


%% Plots
% Plot eigenvalues of the closed-loop system for different speeds
if plots(1)==1
    velocity_eig = linspace(0.001,10,100); % velocity_lqr = linspace(0.001,10,1000), 
                                           % reduced to increase speed of init
    K_full_dis_eig = zeros(length(velocity_eig),3);
    closed_loop_eigenvalues = zeros(3,length(velocity_eig));

    for i = 1:length(velocity_eig)
        v = velocity_eig(i);

        A = [0      0            1;
             0      0            0;
             g/h_real    sin(lambda_real)*(h_real*v^2-g*a_real*c_real)/(h_real^2*b_real)   0;
             0      1            0      0];
        B = [0; 1; (a_real*v*sin(lambda_real))/(h_real*b_real); 0];

        sys_con = ss(A,B,C,D);
        sys_dis = c2d(sys_con,Ts);

        K = lqrd(A,B,Q,R,Ts);     
        lqr_gains_eig(i,:) = K;

        closed_loop_eigenvalues(:,i) = eig(sys_dis.A-sys_dis.B*K);
    end
    
    eigenvalues_abs = abs(closed_loop_eigenvalues);
    eigenvalues_abs = sort(eigenvalues_abs,1);
    
    figure()
    plot(velocity_eig,eigenvalues_abs(1,:))
    hold on
    plot(velocity_eig,eigenvalues_abs(2,:))
    plot(velocity_eig,eigenvalues_abs(3,:))
    grid on
    %title('Eigenvalues of the Closed Loop System vs Veloctity [m/s]')
    xlabel('Velocity [m/s]')
    ylabel('Magnitude of the Eigenvalues')
end


% Plot LQR gains for different speeds
if plots(2)==1
    velocity_lqr = linspace(1.5,10,1000); % velocity_lqr = linspace(0.001,10,1000), 
                                        % reduced to increase speed of init
    lqr_gains_eig_plot = zeros(length(velocity_lqr),3);
    closed_loop_eigenvalues = zeros(3,length(velocity_lqr));

    for i = 1:length(velocity_lqr)
        v = velocity_lqr(i);

        A = [0      0            1;
             0      0            0;
             g/h_real    sin(lambda_real)*(h_real*v^2-g*a_real*c_real)/(h_real^2*b_real)   0;
             0      1            0      0];
        B = [0; 1; (a_real*v*sin(lambda_real))/(h_real*b_real); 0];

        sys_con = ss(A,B,C,D);
        sys_dis = c2d(sys_con,Ts);

        K = lqrd(A,B,Q,R,Ts);     
        lqr_gains_eig_plot(i,:) = K;

    end
    
    figure()
    subplot(3,1,1)
    plot(velocity_lqr,lqr_gains_eig_plot(:,1))
    ylabel('$K_\varphi$','interpreter','latex')
    grid on
    subplot(3,1,2)
    plot(velocity_lqr,lqr_gains_eig_plot(:,2))
    ylabel('$K_\delta$','interpreter','latex')
    grid on
    subplot(3,1,3)
    plot(velocity_lqr,lqr_gains_eig_plot(:,3))
    xlabel('Velocity [m/s]')
    ylabel('$K_{\dot{\varphi}}$','interpreter','latex')
    grid on
end