clc
clear
close all

m = 2;
%跟随者位置状态
p_f1(1,:) = [0,1];
p_f2(1,:) = [0,0];
p_f3(1,:) = [1,0];

%跟随者速度状态

v_f1(1,:) = [0,0];
v_f2(1,:) = [0,0];
v_f3(1,:) = [0,0];

%领航者位置状态
p_l(1,:) = [2,2];

%领航者速度状态
v_l(1,:) = [0,0];

%距离限制
d_d = 1;
%角度关  规定01为1指向0
theta10(1,1) = atan2(p_l(1,2)-p_f1(1,2),p_l(1,1)-p_f1(1,1));
theta30(1,1) = atan2(p_l(1,2)-p_f3(1,2),p_l(1,1)-p_f3(1,1));
theta21(1,1) = atan2(p_f1(1,2)-p_f2(1,2),p_f1(1,1)-p_f2(1,1));
theta13(1,1) = atan2(p_f3(1,2)-p_f1(1,2),p_f3(1,1)-p_f1(1,1));
theta23(1,1) = atan2(p_f3(1,2)-p_f2(1,2),p_f3(1,1)-p_f1(1,1));

dt = 0.01;
%距离测量
d10(1,1) = sqrt((p_l(1,1)-p_f1(1,1))^2+(p_l(1,2)-p_f1(1,2))^2);
d30(1,1) = sqrt((p_l(1,1)-p_f3(1,1))^2+(p_l(1,2)-p_f3(1,2))^2);
d21(1,1) = sqrt((p_f2(1,1)-p_f1(1,1))^2+(p_f2(1,2)-p_f1(1,2))^2);
d13(1,1) = sqrt((p_f1(1,1)-p_f3(1,1))^2+(p_f1(1,2)-p_f3(1,2))^2);
d23(1,1) = sqrt((p_f2(1,1)-p_f3(1,1))^2+(p_f2(1,2)-p_f3(1,2))^2);
%误差关系
e_d10 = d_d - d10(1,1);
e_d30 = d_d - d30(1,1);
e_d21 = d_d - d21(1,1);
e_d13 = d_d - d13(1,1);
e_d23 = d_d - d23(1,1);

e2_d10 = e_d10;
e2_d30 = e_d30;
e2_d21 = e_d21;
e2_d13 = e_d13;
e2_d23 = e_d23;

u_f10(1,:) = [0,0];
u_f30(1,:) = [0,0];
u_f21(1,:) = [0,0];
u_f13(1,:) = [0,0];
u_f23(1,:) = [0,0];

kp_f = 1.15;
kd_f = 5;

figure (1)
hold on
n = 1000;
for i = 2:n
    d10(i,1) = sqrt((p_l(i-1,1)-p_f1(i-1,1))^2+(p_l(i-1,2)-p_f1(i-1,2))^2);
    d30(i,1) = sqrt((p_l(i-1,1)-p_f3(i-1,1))^2+(p_l(i-1,2)-p_f3(i-1,2))^2);
    d21(i,1) = sqrt((p_f2(i-1,1)-p_f1(i-1,1))^2+(p_f2(i-1,2)-p_f1(i-1,2))^2);
    d13(i,1) = sqrt((p_f1(i-1,1)-p_f3(i-1,1))^2+(p_f1(i-1,2)-p_f3(i-1,2))^2);
    d23(i,1) = sqrt((p_f2(i-1,1)-p_f3(i-1,1))^2+(p_f2(i-1,2)-p_f3(i-1,2))^2);
    
    theta10(i,1) = atan2(p_l(i-1,2)-p_f1(i-1,2),p_l(i-1,1)-p_f1(i-1,1));
    theta30(i,1) = atan2(p_l(i-1,2)-p_f3(i-1,2),p_l(i-1,1)-p_f3(i-1,1));
    theta21(i,1) = atan2(p_f1(i-1,2)-p_f2(i-1,2),p_f1(i-1,1)-p_f2(i-1,1));
    theta13(i,1) = atan2(p_f3(i-1,2)-p_f1(i-1,2),p_f3(i-1,1)-p_f1(i-1,1));
    theta23(i,1) = atan2(p_f3(i-1,2)-p_f2(i-1,2),p_f3(i-1,1)-p_f1(i-1,1));
    
    e_d10 = d_d - d10(i,1);
    e_d30 = d_d - d30(i,1);
    e_d21 = d_d - d21(i,1);
    e_d13 = d_d - d13(i,1);
    e_d23 = d_d - d23(i,1);

    ev10_d = (e2_d10 - e_d10)/dt;
    ev30_d = (e2_d30 - e_d30)/dt;
    ev21_d = (e2_d21 - e_d21)/dt;
    ev13_d = (e2_d13 - e_d13)/dt;
    ev23_d = (e2_d23 - e_d23)/dt;
    
    e2_d10 = e_d10;
    e2_d30 = e_d30;
    e2_d21 = e_d21;
    e2_d13 = e_d13;
    e2_d23 = e_d23;
    
    u_f10d  = (kp_f * e_d10 - kd_f * ev10_d)/m;
    u_f30d  = (kp_f * e_d30 - kd_f * ev30_d)/m;
    u_f21d  = (kp_f * e_d21 - kd_f * ev21_d)/m;
    u_f13d  = (kp_f * e_d13 - kd_f * ev13_d)/m;
    u_f23d  = (kp_f * e_d23 - kd_f * ev23_d)/m;
    
    
    u_f10(i,1) = -u_f10d * cos(theta10(i,1));
    u_f10(i,2) = -u_f10d * sin(theta10(i,1));
   
    u_f21(i,1) = -u_f21d * cos(theta21(i,1));
    u_f21(i,2) = -u_f21d * sin(theta21(i,1));
    
    u_f13(i,1) = -u_f13d * cos(theta13(i,1));
    u_f13(i,2) = -u_f13d * sin(theta13(i,1));
    
    u_f23(i,1) = -u_f23d * cos(theta23(i,1));
    u_f23(i,2) = -u_f23d * sin(theta23(i,1));
    
    e_f1vx = v_f3(i-1,1);
    e_f1vy = v_f3(i-1,2);
    u_f1vx(i,1) = kp_f * e_f1vx;
    u_f1vy(i,2) = kp_f * e_f1vy;
    
    e_f3vx = v_f3(i-1,1) - v_f2(i-1,1);
    e_f3vy = v_f3(i-1,2) - v_f2(i-1,2);
    u_f3vx(i,1) = kp_f * e_f3vx;
    u_f3vy(i,2) = kp_f * e_f3vy;
    
    
    
    u_f1(i,1) = u_f10(i,1) + u_f13(i,1)+ u_f1vx(i,1);
    u_f1(i,2) = u_f10(i,2) + u_f13(i,2)+ u_f1vy(i,2);
    
    u_f2(i,1)= u_f21(i,1) + u_f23(i,1) + u_f3vx(i,1);
    u_f2(i,2)= u_f21(i,2) + u_f23(i,2) + u_f3vy(i,2);
    
    u_f30(i,1) = -u_f30d * cos(theta30(i,1));
    u_f30(i,2) = -u_f30d * sin(theta30(i,1));
    
    v_f1(i,1) = v_f1(i-1,1) + u_f1(i,1) * dt;
    v_f1(i,2) = v_f1(i-1,2) + u_f1(i,2) * dt;
    
    v_f3(i,1) = v_f3(i-1,1) + u_f30(i,1) * dt;
    v_f3(i,2) = v_f3(i-1,2) + u_f30(i,2) * dt;
    
    v_f2(i,1) = v_f2(i-1,1) + u_f2(i,1) * dt;
    v_f2(i,2) = v_f2(i-1,2) + u_f2(i,2) * dt;
    
    p_f1(i,1) = p_f1(i-1,1) + 1/2*(v_f1(i,1) + v_f1(i-1,1)) * dt;
    p_f1(i,2) = p_f1(i-1,2) + 1/2*(v_f1(i,2) + v_f1(i-1,2)) * dt;
    
    p_f2(i,1) = p_f2(i-1,1) + 1/2*(v_f2(i,1) + v_f2(i-1,1)) * dt;
    p_f2(i,2) = p_f2(i-1,2) + 1/2*(v_f2(i,2) + v_f2(i-1,2)) * dt;
    
    p_f3(i,1) = p_f3(i-1,1) + 1/2*(v_f3(i,1) + v_f3(i-1,1)) * dt;
    p_f3(i,2) = p_f3(i-1,2) + 1/2*(v_f3(i,2) + v_f3(i-1,2)) * dt;
    
    p_l(i,1) = p_l(i-1,1);
    p_l(i,2) = p_l(i-1,2);
    
    plot(p_f1(i,1),p_f1(i,2),'rx')
    plot(p_f2(i,1),p_f2(i,2),'bx')
    plot(p_f3(i,1),p_f3(i,2),'gx')
    plot(p_l(i,1),p_l(i,2),'ro')
    
end



figure (2)
t = 0:dt:(n-1)*dt;
subplot(231),plot(t,d10);
subplot(232),plot(t,d30);
subplot(233),plot(t,d21);
subplot(234),plot(t,d23);
subplot(235),plot(t,d13);



