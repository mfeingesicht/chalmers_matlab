close all 
%%
        fig1 = figure()

        xlabel('Real')
        ylabel('Imag')
        title('Bike Model Poles Continuous')
        grid on
        hold on
        
        fig2 = figure()
        xlabel('Real')
        ylabel('Imag')
        title('closed loop Poles Discrete')
        hold on
        grid on
%         variable_list = 3.5;           % distance from rear wheel to frame's center of mass [m]
        variable_list = [0.01 1 100] ;           % R

        lgd_des = 'R = ';
        
        lgd = cell(length(variable_list),1);
        sysc = cell(length(variable_list),1);
        sysd_fb = cell(length(variable_list),1);
        for i = 1:length(variable_list)
            v = 5;

            r_wheel = 0.311; % Radius of the wheel
            % Global system (frame+box as 1 volume)
            h_real = 0.2085 + r_wheel;           % height of center of mass [m]
%             h_real = variable_list(i);           % height of center of mass [m]
            b_real = 1.095;            % length between wheel centers [m]
            c_real = 0.06;             % length between front wheel contact point and the 
                                       % extention of the fork axis [m]
            lambda_real = deg2rad(70); % angle of the fork axis [deg]
            a_real = 0.4964;           % distance from rear wheel to frame's center of mass [m]
%             a_real = variable_list(i);
            IMU_height_real = 0.45;    % IMU height [m]
            m = 50;
            g = 9.81;                  % gravity [m/s^2]
            Ts = 0.04;
            J = m*h_real^2;
            D_inertia = m*a_real*h_real;

            A = [0    m*g*h_real/J
                 1          0];
            B = [1; 0];
            C = [v*D_inertia/(b_real*J) m*v^2*h_real/(b_real*J)];
            D = 0;
            sys_con = ss(A,B,C,D);
            sys_dis = c2d(sys_con,Ts);

            Q_phi = [100 0 
                   0 1e-6];
%             R = 10;
            R_phi = variable_list(i); 
            C_bar = [ C; C*A];
            D_bar = [zeros(size(C*B)); C*B];
            
            Q_new = C_bar.'*Q_phi*C_bar;
            N_new = C_bar.'*Q_phi*D_bar;
            R_new = D_bar.'*Q_phi*D_bar + R_phi;
            
            State_trans1 = C_bar^-1;
            State_trans2 = -State_trans1 * D_bar;
            K = lqrd(A,B,Q_new,R_new,N_new,Ts);
%             lqr_gains(1,:) = K
%             K = (-(sys_con.A - sys_con.B * K)^-1 * sys_con.B * K)
%             K_r = 1/(sys_dis.C * (sys_dis.A - sys_dis.B * K)^-1 * sys_dis.B)
%             K_r = (sys_dis.C * sys_dis.B * K) ^ -1 * (1 - sys_dis.C * (sys_dis.A - sys_dis.B * K) * sys_dis.C^-1 )
            K_R = [sys_dis.C * (sys_dis.B * K - sys_dis.A + eye(2))^-1 * sys_dis.B]^-1
%             K_R = K;
%             K_r_inv = (-(sys_con.A - sys_con.B * K_con)^-1 * sys_con.B * K_con);
%             K(3) = 0;

            sysc{i} = ss(A,B,C,D);
            sysd_fb{i} = ss(sys_dis.A-sys_dis.B*K,sys_dis.B*K_R,sys_dis.C,0,Ts);
            sysd_ol{i} = ss(sys_dis.A,sys_dis.B*K_R,sys_dis.C,0,Ts);
%             sysd_fb = d2c(sysd_fb);
            pole_sys{i} = pole(sysc{i});
            pole_sysfb{i} = pole(sysd_fb{i});
            figure(fig1)
            plot(real(pole_sys{i}),imag(pole_sys{i}),'o')
            hold on
            figure(fig2)
            plot(real(pole_sysfb{i}),imag(pole_sysfb{i}),'o')
            lgd{i} = [ lgd_des , num2str(variable_list(i))];
            
            
        end
        legend(lgd)
                    figure(fig1)
                    legend(lgd)
         ref = deg2rad(1);
 %%       
        fig3 = figure()
        input = ['pzplot('];
        for i = 1:length(sysc)

            input = [input 'sysc{' num2str(i) '},'];
            hold on
        end
        input(end) = ')'
        eval(input)
        legend(lgd)
        title('Zero-Pole Map BIKE MODEL Continuous')
        
                fig4 = figure()
        input = ['pzplot('];
        for i = 1:length(sysc)

            input = [input 'sysd_fb{' num2str(i) '},'];
            hold on
        end
        input(end) = ')'
        eval(input)
        legend(lgd)
        title('Zero-Pole Map CLOSED LOOP Discrete')
        
        %% Bode Plot Closed Loop
        fig5 = figure()
        input = ['bode('];
        for i = 1:length(sysc)

            input = [input 'sysd_fb{' num2str(i) '},'];
            hold on
        end
        input(end) = ')'
        eval(input)
        legend(lgd)
        title('Bode Plot CLOSED LOOP Discrete')
        
        %% Bode Plot with Margin Info OPEN LOOP
        fig6 = figure()
        


        [r c] = size(sysd_ol{1});
        hold on
        for j = 1:r*c
            subplot(r,c,j)
            for i = 1:length(variable_list)
                hold on
                margin(sysd_ol{i}(ceil(j/c), mod(j - ceil(j/c  - 1)*c,c+1)))
            end
            if i > 1
                title('Phase and Gain Margin Bode Plots')
                legend(lgd)
            end
        end
        hand_t=suptitle('Gain and Phase Margin of the OPEN LOOP SYSTEM ref -> y');


        %% Sensitivity PLOT CLOSED LOOP System
        fig7 = figure()

        [r c] = size(sysd_ol{1});
        hold on
        for j = 1:r*c
            subplot(r,c,j)
            for i = 1:length(variable_list)
                hold on
                bode( 1/( 1 + tf(sysd_ol{i}(ceil(j/c), mod(j - ceil(j/c  - 1)*c,c+1)) ) ) )
            end
            title(['Sensitivity input :'  num2str( mod(j - ceil(j/c - 1)*c,c+1) ) 'output:' num2str(ceil(j/c))])
            legend(lgd)
        end
        hand_t=suptitle('Sensitivity Functions CLOSED LOOP e -> ref');
        
        %% Nyquist
        fig8 = figure()
        hold on

            for i = 1:length(variable_list)
                hold on
                nyquist(sysd_ol{i})

            end
                        legend(lgd)
%         hand_t=suptitle('Sensitivity Functions CLOSED LOOP e -> ref');
        %% SIMULINK File
        simulink_file = 'LQR_Trial_new20191002.slx';
        sim(simulink_file)