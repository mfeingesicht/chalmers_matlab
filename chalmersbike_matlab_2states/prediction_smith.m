A_bike_dis;
B_bike_dis;
d_delay = 3;%delay_tf_steering/Ts;

vec_powers_A = [];
vec_powers_AB = [];
for i=1:d_delay
    vec_powers_A = [vec_powers_A , A_bike_dis^(d_delay-i)];
    vec_powers_AB = [vec_powers_AB , A_bike_dis^(d_delay-i)*B_bike_dis];
end
A_bike_dis_prediction = A_bike_dis^d_delay;
B_bike_dis_prediction = vec_powers_AB;