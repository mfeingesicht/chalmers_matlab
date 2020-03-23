function [cost] = objfun_currentControl(Jf,P_all,omega_all)
    % Bike parameters
    Jm = 72.8*1e-7; % Rotor inertia (kg.m2)
    kt = 27.3*1e-3; % torque constant (Nm/A)
    gr_b = 1;       % Belt gear ratio
    gr_sm = 111;    % Steering motor gear ratio
    N = gr_sm*gr_b;

    J = Jm + Jf/N^2;
    s = tf('s');
    sys = kt / (N*J*s^2);

    [P_tf_all,delta_tf_all] = bode(sys,omega_all);
    P_tf_all = squeeze(P_tf_all);
    delta_tf_all = squeeze(delta_tf_all);

    cost = norm(P_all'-P_tf_all);
end