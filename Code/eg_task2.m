clear all

m = 2;
p_f1(1,:) = [0,0];
p_f2(1,:) = [0,1];
p_f3(1,:) = [1,0];
p_l(1,:) = [2,2];
v_f1(1,:) = [0,0];
v_f2(1,:) = [0,0];
v_f3(1,:) = [0,0];
v_l(1,:) = [0,0];
v_ll(1,1) = 0;
v_ff(1,1) = 0;
d_d = 1;
dt = 0.01;

theta_l(1,1) = 30/180*pi;
theta1L(1,1) = atan2(p_l(1,2)-p_f1(1,2),p_l(1,1)-p_f1(1,1));
theta2(1,1) = atan2(p_l(1,2)-p_f2(1,2),p_l(1,1)-p_f2(1,1));
theta3(1,1) = atan2(p_l(1,2)-p_f3(1,2),p_l(1,1)-p_f3(1,1));
theta1Lxiu(1,1) = theta1L(1,1);

d1f1L(1,1) = sqrt((p_l(1,1)-p_f1(1,1))^2+(p_l(1,2)-p_f1(1,2))^2);%跟随者一号到领航者的初始化
e_d1f1L = d_d - d1f1L(1,1);
e_d2f1L = e_d1f1L;
u_f1(1,:) = [0,0];

d1f2L(1,1) = sqrt((p_l(1,1)-p_f2(1,1))^2+(p_l(1,2)-p_f2(1,2))^2);%跟随者二号到领航者的初始化
e_d1f2L = d_d - d1f2L(1,1);
e_d2f2L = e_d1f2L;
u_f2(1,:) = [0,0];

d1f3L(1,1) = sqrt((p_l(1,1)-p_f3(1,1))^2+(p_l(1,2)-p_f3(1,2))^2);%跟随者三号到领航者的初始化
e_d1f3L = d_d - d1f3L(1,1);
e_d2f3L = e_d1f3L;
u_f3(1,:) = [0,0];

d1f21(1,1) = sqrt((p_f1(1,1)-p_f2(1,1))^2+(p_f1(1,2)-p_f2(1,2))^2);%跟随者2号到1号的初始化
e_d1f21 = d_d - d1f21(1,1);
e_d2f21 = e_d1f21;

d1f31(1,1) = sqrt((p_f1(1,1)-p_f3(1,1))^2+(p_f1(1,2)-p_f3(1,2))^2);%跟随者3号到1号的初始化
e_d1f31 = d_d - d1f31(1,1);
e_d2f31 = e_d1f31;

kp_f1 = 4.7;
kd_f1 = 5;
kp_f2 = 4.7;
kd_f2 = 5;
kp_f3 = 4.7;
kd_f3 = 5;
kp_l = 4.7;

figure (1)
hold on
n = 3000;
for i = 2:n
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    theta_l(i,1) = 30/180*pi*(1+sin(i*dt/2));

    
    v_rl(1,1) = 1.5 * cos(theta_l(i-1,1));
    v_rl(1,2) = 1.5 * sin(theta_l(i-1,1));
    
    e_lvx = v_rl(1,1) - v_l(i-1,1);
    e_lvy = v_rl(1,2) - v_l(i-1,2);
    
    u_l(i,1) = kp_l * e_lvx;
    u_l(i,2) = kp_l * e_lvy;
    
    v_l(i,1) = v_l(i-1,1) + u_l(i,1) * dt;
    v_l(i,2) = v_l(i-1,2) + u_l(i,2) * dt;
    v_ll(i,1) = sqrt(v_l(i,1)^2+v_l(i,2)^2);
    
    p_l(i,1) = p_l(i-1,1) + 1/2*(v_l(i,1) + v_l(i-1,1)) * dt;
    p_l(i,2) = p_l(i-1,2) + 1/2*(v_l(i,2) + v_l(i-1,2)) * dt;
    
    %跟随者1向领航者到距离一米的位置
    d1f1L(i,1) = sqrt((p_l(i-1,1)-p_f1(i-1,1))^2+(p_l(i-1,2)-p_f1(i-1,2))^2);
    theta1L(i,1) = atan2(p_l(i-1,2)-p_f1(i-1,2),p_l(i-1,1)-p_f1(i-1,1));
    
    e_d1f1L = d_d - d1f1L(i,1);
    ev_df1L = (e_d2f1L - e_d1f1L)/dt;
    e_d2f1L = e_d1f1L;
    
    u_f1d  = kp_f1 * e_d1f1L - kd_f1 * ev_df1L;
    
    %跟随者1的速度控制分量
    
    e_f1vx = 1.5 * cos(theta_l(i-1,1)) - v_f1(i-1,1);
    e_f1vy = 1.5 * sin(theta_l(i-1,1)) - v_f1(i-1,2);
    
    u_f1vx(i,1) = kp_f1 * e_f1vx;
    u_f1vy(i,2) = kp_f1 * e_f1vy;
           
    u_f1(i,1) = -u_f1d * cos(theta1L(i,1))+u_f1vx(i,1);
    u_f1(i,2) = -u_f1d * sin(theta1L(i,1))+u_f1vy(i,2);
% %     u_f1(i,1) = -u_f1d * cos(theta_l(i,1));
% %     u_f1(i,2) = -u_f1d * sin(theta_l(i,1));
        
   
    v_f1(i,1) = v_f1(i-1,1) + u_f1(i,1) * dt;
    v_f1(i,2) = v_f1(i-1,2) + u_f1(i,2) * dt;
    
    
    p_f1(i,1) = p_f1(i-1,1) + 1/2*(v_f1(i,1) + v_f1(i-1,1)) * dt;
    p_f1(i,2) = p_f1(i-1,2) + 1/2*(v_f1(i,2) + v_f1(i-1,2)) * dt; 
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %跟随者2跟随领航者的得到的控制分量
    
    d1f2L(i,1) = sqrt((p_l(i-1,1)-p_f2(i-1,1))^2+(p_l(i-1,2)-p_f2(i-1,2))^2);
    theta2L(i,1) = atan2(p_l(i-1,2)-p_f2(i-1,2),p_l(i-1,1)-p_f2(i-1,1));
    
    e_d1f2L = d_d - d1f2L(i,1);
    ev_df2L = (e_d2f2L - e_d1f2L)/dt;
    e_d2f2L = e_d1f2L;
    
    u_f2Ld  = kp_f2 * e_d1f2L - kd_f2 * ev_df2L;
    
    %跟随者2跟随跟随者1的得到的控制分量
    d1f21(i,1) = sqrt((p_f1(i-1,1)-p_f2(i-1,1))^2+(p_f1(i-1,2)-p_f2(i-1,2))^2);
    theta21(i,1) = atan2(p_f1(i-1,2)-p_f2(i-1,2),p_f1(i-1,1)-p_f2(i-1,1));
    
    e_d1f21 = d_d - d1f21(i,1);
    ev_df21 = (e_d2f21 - e_d1f21)/dt;
    e_d2f21 = e_d1f21;
    
    u_f21d  = kp_f2 * e_d1f21 - kd_f2 * ev_df21;
    
    %跟随者2跟随跟随者1速度得到的控制分量
    e_f2vx = 1.5 * cos(theta_l(i-1,1)) - v_f2(i-1,1);
    e_f2vy = 1.5 * sin(theta_l(i-1,1)) - v_f2(i-1,2);
    
    u_f2vx(i,1) = kp_f2 * e_f2vx;
    u_f2vy(i,2) = kp_f2 * e_f2vy;
    
    %跟随者2跟随领航者和跟随者1的总跟随控制量
    u_f2(i,1) = -u_f2Ld * cos(theta2L(i,1))-u_f21d * cos(theta21(i,1))+u_f2vx(i,1);
    u_f2(i,2) = -u_f2Ld * sin(theta2L(i,1))-u_f21d * sin(theta21(i,1))+u_f2vy(i,2);
        
    
    v_f2(i,1) = v_f2(i-1,1) + u_f2(i,1) * dt;
    v_f2(i,2) = v_f2(i-1,2) + u_f2(i,2) * dt;
    

    p_f2(i,1) = p_f2(i-1,1) + 1/2*(v_f2(i,1) + v_f2(i-1,1)) * dt;
    p_f2(i,2) = p_f2(i-1,2) + 1/2*(v_f2(i,2) + v_f2(i-1,2)) * dt;
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %跟随者3跟随领航者的得到的控制分量
    d1f3L(i,1) = sqrt((p_l(i-1,1)-p_f3(i-1,1))^2+(p_l(i-1,2)-p_f3(i-1,2))^2);
    theta3L(i,1) = atan2(p_l(i-1,2)-p_f3(i-1,2),p_l(i-1,1)-p_f3(i-1,1));
    
    e_d1f3L = d_d - d1f3L(i,1);
    ev_df3L = (e_d2f3L - e_d1f3L)/dt;
    e_d2f3L = e_d1f3L;
    
    u_f2Ld  = kp_f3 * e_d1f3L - kd_f3 * ev_df3L;
    
    %跟随者3跟随跟随者1的得到的控制分量
    d1f31(i,1) = sqrt((p_f1(i-1,1)-p_f3(i-1,1))^2+(p_f1(i-1,2)-p_f3(i-1,2))^2);
    theta31(i,1) = atan2(p_f1(i-1,2)-p_f3(i-1,2),p_f1(i-1,1)-p_f3(i-1,1));
    
    e_d1f31 = d_d - d1f31(i,1);
    ev_df31 = (e_d2f31 - e_d1f31)/dt;
    e_d2f31 = e_d1f31;
    
    u_f21d  = kp_f3 * e_d1f31 - kd_f3 * ev_df31;
    
      %跟随者3跟随跟随者1的得到的速度控制分量
    
    e_f3vx = 1.5 * cos(theta_l(i-1,1)) - v_f3(i-1,1);
    e_f3vy = 1.5 * sin(theta_l(i-1,1)) - v_f3(i-1,2);
    
    u_f3vx(i,1) = kp_f3 * e_f3vx;
    u_f3vy(i,2) = kp_f3 * e_f3vy;
    
    
    %跟随者3跟随领航者和跟随者1的总跟随控制量
    u_f3(i,1) = -u_f2Ld * cos(theta3L(i,1))-u_f21d * cos(theta31(i,1))+ u_f3vx(i,1);
    u_f3(i,2) = -u_f2Ld * sin(theta3L(i,1))-u_f21d * sin(theta31(i,1))+u_f3vy(i,2) ;
        
    
    v_f3(i,1) = v_f3(i-1,1) + u_f3(i,1) * dt;
    v_f3(i,2) = v_f3(i-1,2) + u_f3(i,2) * dt;
    

    p_f3(i,1) = p_f3(i-1,1) + 1/2*(v_f3(i,1) + v_f3(i-1,1)) * dt;
    p_f3(i,2) = p_f3(i-1,2) + 1/2*(v_f3(i,2) + v_f3(i-1,2)) * dt;
    
    plot(p_f1(i,1),p_f1(i,2),'rx')
    plot(p_f2(i,1),p_f2(i,2),'bx')
    plot(p_f3(i,1),p_f3(i,2),'gx')
    plot(p_l(i,1),p_l(i,2),'yo')
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
t = 0:dt:(n-1)*dt;
figure(2);
subplot(231),plot(t,d1f1L);
subplot(232),plot(t,d1f21);
subplot(233),plot(t,d1f2L);
subplot(234),plot(t,d1f31);
subplot(235),plot(t,d1f3L);

