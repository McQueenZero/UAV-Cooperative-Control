%%-------------------------------------------------------------------------
% 作者：   赵敏琨
% 学号：   2018302068
% 时间：   2021年12月
%%-------------------------------------------------------------------------
clc
clear
close all
%% 初始化
% 领航者状态变量
x1 = [1000 1000 20];  % x, y, v
X(:, :, 1) = [x1'];

v_r = 180 / 3.6;  % 期望参考速度
% 航迹状态量
xw1 = [90/180*pi 0];  % theta, w
W(:, :, 1) = [xw1'];

a1 = [0 0];  % alon, alat
U(:, :, 1) = [a1'];  % 领航者控制变量

rx = 1400:300:3800;
ry = tand(90-75) * (rx-1000) + 1000;
rp = [rx' ry'];  % L1参考点
jj = 1;

dt = 0.01;  % 采样时间
kw = 2;  % 角速度控制比例系数
kv = 0.3;  % 速度控制比例系数
t = 0:dt:50;

%% 实时解算
figure
hold on
n = length(t);
plot(rp(:,1), rp(:,2), 'kx', 'MarkerSize', 6, 'LineWidth', 1)
for k = 2:n 
    l1d = norm(X(1:2, 1, k-1)-rp(jj, :)');  % L1距离
    dtheta = atan((X(2, 1, k-1)-rp(jj, 2)) / (X(1, 1, k-1)-rp(jj, 1)));  % 目标点与无人机间的方向角
    eta = dtheta - W(1, 1, k-1);
    
    if (rp(jj,1)-X(1, 1, k-1)) < 0
        dtheta = dtheta + pi;
    elseif (rp(jj,2)-X(2, 1, k-1)) < 0
        dtheta = dtheta + pi*2;        
    end
    
    e_V = X(3, 1, k-1) - v_r;
    U(1, 1, k) = -kv * e_V;  % 纵向加速度
    U(2, 1, k) = 2 * X(3, 1, k-1)^2 / l1d * sin(eta);  % 横向加速度
    
    W(2, 1, k) = kw * U(2, 1, k) / X(3, 1, k-1);  % 更新角速度
    W(1, 1, k) = W(1, 1, k-1) + W(2, 1, k) * dt;  % 更新航向角
    
    % L1制导 更新横向加速度、角速度、航向角
    Phi_dp = [0 0 dt*cos(W(1, 1, k))
              0 0 dt*sin(W(1, 1, k))
              0 0                 0 ];
    Phi = eye(3) + Phi_dp;
    Gam = [0 0; 0 0; dt 0];

    % 状态方程 更新位置及纵向加速度
    % [x       [1 0 dt*cos(theta)    [x         [0  0    [alon
    %  y     =  0 1 dt*sin(theta)  *  y       +  0  0  *  alat] 
    %  v](k)    0 0            1 ]    v](k-1)    dt 0]
    X(:, 1, k) =  Phi * X(:, 1, k-1) + Gam * U(:, 1, k-1);
    x1(k, :) = X(:, 1, k)';
    a1(k, :) = U(:, 1, k)';
    xw1(k, :) = W(:, 1, k)';
        
    % 实时画图
%     plot(X(1, 1, k), X(2, 1, k), 'r.')  
%     pause(0.001)
    
    if l1d < 1  % 更新目标点
        if jj < length(rp)
            jj = jj + 1;
        else
%             break % 到达最后目标点后退出
        end
    end    
end
% xlabel('x/m','fontsize',12); ylabel('y/m','fontsize',12);
% legend({'L1导航点', '领航者1'},'Location','best');

%% 根据记录数据画图
plot(x1(:, 1), x1(:, 2), 'LineWidth', 1)
grid on
xlabel('x/m','fontsize',12); ylabel('y/m','fontsize',12);
title('轨迹曲线')
legend({'L1导航点', '领航者1'},'Location','best')

figure
plot(t, x1(:, 3), 'LineWidth', 1)
grid on
xlabel('t/s','fontsize',12); ylabel('v/m/s','fontsize',12);
title('速度曲线')

figure
hold on
plot(t, rad2deg(xw1(:, 1)), 'LineWidth', 1)
plot(t, rad2deg(xw1(:, 2)), 'LineWidth', 1)
grid on
xlabel('t/s','fontsize',12); ylabel('\theta/deg \omega/deg/s','fontsize',12);
title('航向角/角速度曲线')
legend({'航向角', '角速度'},'Location','best')
