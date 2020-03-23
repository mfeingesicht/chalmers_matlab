function [u,eta,xk] =  mpc_Setpoint(x0,xk0,u0,Nc,Np,r,Delta_t,v)
% x0 = zeros(3,1);
% xk0 = zeros(6,1);
% u0 = 0;
% Nc = 25;
% Np = 25;
% r = zeros(3,1);
% Delta_t = 0.025;
% v = 3;
% This Function export the Optimal MPC estimation based on the states
% It runs at timestep k (so now),calculate the signal input for next timestep.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% 
% Input: 
% -> x0 := The current(k_{th} step) states column (state-space model), includes
% [x0;y0;Psi0;u;v;w;]
% -> xk0 := The Augmented System states, includes [\delta x_m(k); y(k)]
% -> Nc := The control horizon, which means, take actions(manipulate the
% input signal in the next Nc steps)
% -> Np := Prediction Horizon,estimate the states in the next Np steps;ß
% -> r := The reference signal, indicating which states you want to
% achieve,[x_d;y_d;Psi_d;u_d;v_d;w_d;]; 
% -> Delta_t := The sample time for discritization
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% 
% Output:
% -> eta := The optimal \delta U input signal for the system in next Nc Steps,
% [\delta_TauX1;\delta_TauY1;\delta_TauPsi1;\delta_TauX2;\delta_TauY2;...;TauXNc;TauYNc;TauPsiNc];
% -> xk  := The estimation of xk,which may be used for comparison how good
% it tracks the trajectory
 
%% Initialization





% u = zeros(3,1);



%%
[A_m,B_m] = Lin_Sys(v);

C_m = eye(3);
D_m = zeros(3,1);
% The Input Increment Setting

R_bar = zeros(Nc);

% The Input Weight setting
% 
K_u      = 10*eye(Nc,Nc);
% for i = 1:Nc
%    K_u(i,i) = 10;
% end

K_bar = zeros(Np*3);
for i = 1:Np
   K_bar((3)*i-2,(3)*i-2) = 1e2./(1e-2*Kx_gain+1e-3); % The Weight for \Phi
   K_bar((3)*i-1,(3)*i-1) = 1e2./(1e-2*Ky_gain+1e-3); % The Weight for Steering \delta
   K_bar((3)*i,(3)*i)     = 1e3./(1e-2*Yaw_gain+1e-3); % The Weight for \dot{\Phi}
end

K_bar((3)*i-2,(3)*i-2) = 10*K_bar((3)*i-2,(3)*i-2) ; % The terminal weight for \Phi
K_bar((3)*i-1,(3)*i-1) = 10*K_bar((3)*i-1,(3)*i-1) ; % The Terminal Weight for Steering \delta
K_bar((3)*i,(3)*i)     = 1000*K_bar((3)*i,(3)*i) ;  % The Terminal Weight for \dot{\Phi}
% Rs = ones(Np,3)*r;

% MagSurge = 400*1e4;
% MagSway  = 100*1e4;
% MagYaw   = 1e6*1e4;
% DeltaSurge = 5e4;
% DeltaSway  = 5e4;
% DeltaYaw   = 5e7;
% 
% MagSurge = 100*1e4;
% MagSway  = 100*1e4;
% MagYaw   = 1e6*1e4;
% DeltaSurge = 5e4;
% DeltaSway  = 5e4;
% DeltaYaw   = 5e7;
% MagSurge = 1e4;
% MagSway  = 1e4;
% MagYaw   = 1e4;
% 
% end


%% Discretization
coder.extrinsic('c2dm','sum');

[Ap,Bp,Cp,~]=c2dm(A_m,B_m,C_m,D_m,Delta_t,'zoh');

%% Augumented System 
[Phi_Phi,Phi_F,Phi_R,A_e,B_e,~,~,~] = mpcgain(Ap,Bp,Cp,Nc,Np,r,K_bar);


%% solve the optimization problem

% H0 = 2*(Phi_Phi + R_bar); % 3*Nc by 3*Nc
% f0 = -2*(Phi_R-Phi_F*xk0); % 3*Nc by 1
% 
% 
% % The Positive Part for Manipulated Variable(Input) Constraints
% A_cons1 = eye(3*Nc,3*Nc);
% B_cons1 = zeros(3*Nc,1);
% for j = 1:Nc % The #column
%     for i = 1+j:Nc % The #row 
%     A_cons1(3*i-2:3*i,3*j-2:3*j) = eye(3);
% %     A_cons(3*(Nc+i)-2:3*(Nc+i),3*j-2:3*j) = -eye(3);
%     end
%     B_cons1(3*j-2:3*j,1) = [MagSurge;MagSway;MagYaw]-u0;
% %     B_cons(3*(Nc+i)-2:3*(Nc+i),1) =[10000*1e4;10000*1e4;1e4*1e5]+u0;
% end
% 
% % The Negative Part for Manipulated Variable(Input) Constraints
% A_cons2 = eye(3*Nc,3*Nc);
% B_cons2 = zeros(3*Nc,1);
% 
% for j = 1:Nc % The #column
%     for i = 1+j:Nc % The #row 
%     A_cons2(3*i-2:3*i,3*j-2:3*j) = -eye(3);
% %     A_cons2(3*(Nc+i)-2:3*(Nc+i),3*j-2:3*j) = -eye(3);
%     end
%     B_cons2(3*j-2:3*j,1) = [MagSurge;MagSway;MagYaw]+u0;
% %     B_cons2(3*(Nc+i)-2:3*(Nc+i),1) =[10000*1e4;10000*1e4;1e4*1e5]+u0;
% end
% 
% % The Constraints for Input Increment Positive Part
% 
% A_cons3 = eye(3*Nc,3*Nc);
% B_cons3 = zeros(3*Nc,1);
% 
% Wr = 1e3;  % The Multipiler for Increment Input
% 
% for i = 1:Nc % The #row
%     B_cons3(3*i-2:3*i,1) = [DeltaSurge;DeltaSway;DeltaYaw]*Wr;
% end
% 
% % The Constraints for Input Increment Negative Part
% 
% A_cons4 = -eye(3*Nc,3*Nc);
% B_cons4 = zeros(3*Nc,1);
% 
% for i = 1:Nc % The #row
%     B_cons4(3*i-2:3*i,1) = [DeltaSurge;DeltaSway;DeltaYaw]*Wr;
% end


% Constructing the 3Nc*3Nc Lower-Diagnol Identity Matrix P:
%      1     0     0     0     0     0     
%      0     1     0     0     0     0     
%      0     0     1     0     0     0     
%      1     0     0     1     0     0     
%      0     1     0     0     1     0     
%      0     0     1     0     0     1 3Nc*3Nc This is also useful for
%      constrainting the input U

% % The Input sequence as 
% % [ u0;u0;...]Nc*1
P = eye(Nc,Nc);
U0= zeros(Nc,1);
% 
% A_cons1 = zeros(Nc,Nc);
% B_cons1 = zeros(Nc,1);
% A_cons2 = zeros(Nc,Nc);
% B_cons2 = zeros(Nc,1);
% A_cons3 = eye(Nc,Nc);
% B_cons3 = zeros(Nc,1);
% A_cons4 = -eye(Nc,Nc);
% B_cons4 = zeros(Nc,1);
% 
% 
for j = 1:Nc % The #column
    for i = 1+j:Nc % The #row 
    P(i,j) = 1;
    end
    U0(j) = u0;
%     B_cons1(3*j-2:3*j,1) = [MagSurge;MagSway;MagYaw]-u0;
%     B_cons2(3*j-2:3*j,1) = [MagSurge;MagSway;MagYaw]+u0;
%     B_cons3(3*j-2:3*j,1) = [DeltaSurge;DeltaSway;DeltaYaw]*Wr;
end
% B_cons4 = B_cons3;
% A_cons1 = P;
% A_cons2 = -P;



H0 = 2*(Phi_Phi + R_bar+P.'*K_u*P); % Nc by Nc
f0 = 2*(-Phi_R+Phi_F*xk0+P.'*K_u*U0); % Nc by 1

% 
% A_cons = [A_cons1;A_cons2;A_cons3;A_cons4];
% B_cons = [B_cons1;B_cons2;B_cons3;B_cons4];
% 
% 
% A_cons = [A_cons1;A_cons2;];
% B_cons = [B_cons1;B_cons2;];

% B_cons1 = double(B_cons1);
x=-H0\f0; % Calculate an Optimal Solution WITHOUT CONSTRAINTS
% x = QPhild(H0,f0,A_cons,B_cons);
% size(B_e)
% [x,fval] = quadprog(H0,f0,[],[],[],[]);
% [x,fval] = quadprog(H0,f0,A_cons,B_cons,A_Eq1,B_Eq1);
% coder.extrinsic('quadprog');
% x = quadprog(H0,f0,A_cons,B_cons);
eta = x(1); % The force increments to be applied
u = u0 + eta;
% xk = zeros(6,1);
xk = A_e*xk0 + B_e*eta;
end

