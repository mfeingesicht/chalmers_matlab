% Create a vector of velocities at which LQR gains will be computed
velocity_lqr = [input_velocity - 0.1 , input_velocity , input_velocity + 0.1];
% velocity_lqr = input_velocity;

%% Obtain LQR gains
% Initialize gains and eigenvalues matrices
lqr_gains = zeros(length(velocity_lqr),1+size(A_bike,1));

for i = 1:length(velocity_lqr)
    v = velocity_lqr(i);

    A_lqr = A_bike;
    B_lqr = B_bike;
    C_lqr = [C1_bike*v C2_bike*v^2];
    D_lqr = D_bike;
    
    sys_con = ss(A_lqr,B_lqr,C_lqr,D_lqr);
    sys_dis = c2d(sys_con,Ts);
    A_lqr_dis = sys_dis.A;
    B_lqr_dis = sys_dis.B;
    C_lqr_dis = sys_dis.C;
    D_lqr_dis = sys_dis.D;
    
    % Transform Q and R from [phi ; phidot] to x
    Cbar = obsv(A_lqr,C_lqr);
    Dbar = [zeros(size(C_lqr,1),size(B_lqr,2)) ; C_lqr*B_lqr];
    Q_x = Cbar'*Q_phi*Cbar;
    R_x = Dbar'*Q_phi*Dbar+R_phi;
    N_x = (Dbar'*Q_phi*Cbar)';
    
    % lqrd computes discrete LQR gains from continuous time matrices
    % => For use in discrete controller
    K_lqr = lqrd(A_lqr,B_lqr,Q_x,R_x,N_x,Ts);
%     lqr_gains(i,:) = K_lqr;
    lqr_gains(i,:) = [input_velocity K_lqr];
end