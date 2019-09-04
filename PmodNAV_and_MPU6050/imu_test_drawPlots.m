% load('test_noLateralMovement.mat');
load('test_smallLateralMovement.mat');

h = figure;ylim([-90 90]);
hold on;
h1 = plot(time(2:end),roll_comp_vec_MPU6050(2:end),'--m','XDataSource','time','YDataSource','roll_comp_vec');
h2 = plot(time(2:end),roll_comp_latComp_vec_MPU6050(2:end),'--r','XDataSource','time','YDataSource','roll_comp_latComp_vec');
h3 = plot(time(2:end),roll_intGyro_vec_MPU6050(2:end),'--g','XDataSource','time','YDataSource','roll_intGyro_vec');
h4 = plot(time(2:end),roll_madgwick_vec_PmodNAV(2:end),'r','XDataSource','time','YDataSource','roll_madgwick_vec');
h5 = plot(time(2:end),roll_mahony_vec_PmodNAV(2:end),'y','XDataSource','time','YDataSource','roll_mahony_vec');
h6 = plot(time(2:end),roll_comp_vec_PmodNAV(2:end),'m','XDataSource','time','YDataSource','roll_comp_vec');
h7 = plot(time(2:end),roll_intGyro_vec_PmodNAV(2:end),'g','XDataSource','time','YDataSource','roll_intGyro_vec');
h8 = plot(time(2:end),roll_true_vec(2:end),'b','XDataSource','time','YDataSource','roll_true_vec');
legend('Roll complementary MPU6050','Roll complementary with LatComp MPU6050','Roll rate integration MPU6050','Roll Madwick PmodNAV','Roll Mahony PmodNAV','Roll complementary PmodNAV','Roll rate integration PmodNAV','True roll');

% MPU 6050
hh = figure;
hold on;
subplot(321);hh1 = plot(time(2:end),1000*ax_vec_MPU6050(2:end),'XDataSource','time','YDataSource','ax_vec');title('ax MPU6050 (mg)');
subplot(323);hh2 = plot(time(2:end),1000*ay_vec_MPU6050(2:end),'XDataSource','time','YDataSource','ay_vec');title('ay MPU6050 (mg)');
subplot(325);hh3 = plot(time(2:end),1000*az_vec_MPU6050(2:end),'XDataSource','time','YDataSource','az_vec');title('az MPU6050 (mg)');
subplot(322);hh4 = plot(time(2:end),gx_vec_MPU6050(2:end),'XDataSource','time','YDataSource','gx_vec');title('gx MPU6050 (deg/s)');
subplot(324);hh5 = plot(time(2:end),gy_vec_MPU6050(2:end),'XDataSource','time','YDataSource','gy_vec');title('gy MPU6050 (deg/s)');
subplot(326);hh6 = plot(time(2:end),gz_vec_MPU6050(2:end),'XDataSource','time','YDataSource','gz_vec');title('gz MPU6050 (deg/s)');

% PmodNAV
hhh = figure;
hold on;
subplot(321);hhh1 = plot(time(2:end),ax_vec_PmodNAV(2:end),'XDataSource','time','YDataSource','ax_vec');title('ax PmodNAV (mg)');
subplot(323);hhh2 = plot(time(2:end),ay_vec_PmodNAV(2:end),'XDataSource','time','YDataSource','ay_vec');title('ay PmodNAV (mg)');
subplot(325);hhh3 = plot(time(2:end),az_vec_PmodNAV(2:end),'XDataSource','time','YDataSource','az_vec');title('az PmodNAV (mg)');
subplot(322);hhh4 = plot(time(2:end),gx_vec_PmodNAV(2:end),'XDataSource','time','YDataSource','gx_vec');title('gx PmodNAV (deg/s)');
subplot(324);hhh5 = plot(time(2:end),gy_vec_PmodNAV(2:end),'XDataSource','time','YDataSource','gy_vec');title('gy PmodNAV (deg/s)');
subplot(326);hhh6 = plot(time(2:end),gz_vec_PmodNAV(2:end),'XDataSource','time','YDataSource','gz_vec');title('gz PmodNAV (deg/s)');


% Errors
err_comp_MPU6050 = norm(roll_true_vec(2:end) - roll_comp_vec_MPU6050(2:end));
err_comp_latComp_MPU6050 = norm(roll_true_vec(2:end) - roll_comp_latComp_vec_MPU6050(2:end));
err_intGyro_MPU6050 = norm(roll_true_vec(2:end) - roll_intGyro_vec_MPU6050(2:end));
err_magwick_PmodNAV = norm(roll_true_vec(2:end) - roll_madgwick_vec_PmodNAV(2:end));
err_mahony_PmodNAV = norm(roll_true_vec(2:end) - roll_mahony_vec_PmodNAV(2:end));
err_comp_PmodNAV = norm(roll_true_vec(2:end) - roll_comp_vec_PmodNAV(2:end));
err_intGyro_PmodNAV = norm(roll_true_vec(2:end) - roll_intGyro_vec_PmodNAV(2:end));
fprintf('err_comp_MPU6050\t\t\t%f\nerr_comp_latComp_MPU6050\t%f\nerr_intGyro_MPU6050\t\t\t%f\nerr_magwick_PmodNAV\t\t\t%f\nerr_mahony_PmodNAV\t\t\t%f\nerr_comp_PmodNAV\t\t\t%f\nerr_intGyro_PmodNAV\t\t\t%f\n',err_comp_MPU6050,err_comp_latComp_MPU6050,err_intGyro_MPU6050,err_magwick_PmodNAV,err_mahony_PmodNAV,err_comp_PmodNAV,err_intGyro_PmodNAV);