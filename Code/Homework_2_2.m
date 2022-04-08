%%-------------------------------------------------------------------------
% 作者：   赵敏琨
% 学号：   2018302068
% 时间：   2021年12月
%%-------------------------------------------------------------------------
clc
clear
close all
%% 初始化
% 四机状态变量
x1 = [2 2 0 0];  % x, y, vx, vy
x2 = [0 0 0 0];
x3 = [0 1 0 0];
x4 = [1 0 0 0];
X(:, :, 1) = [x1' x2' x3' x4'];
A(:, :, 1) = anglemat(X(:, :, 1));  % 角度邻接矩阵
D(:, :, 1) = distancemat(X(:, :, 1));  % 距离邻接矩阵
D_rec = [D(2, 1, 1) D(3, 1, 1) D(3, 2, 1) ...
         D(4, 1, 1) D(4, 2, 1) D(4, 3, 1)];
d_r = 1;  % 期望参考距离
e_D = d_r - d_r * eye(size(D(:, :, 1))) - D(:, :, 1);  % 距离误差邻接矩阵
e2_D = e_D;

v_r = 1.5;  % 期望参考速度
a_ini = (90-30)/180*pi;  % 初始角度
ei_V = zeros(2, 4);  % 四机速度误差矩阵[V1x V2x ...; V1y V2y ...]

a1 = [0 0]; a2 = [0 0]; a3 = [0 0]; a4 = [0 0];
U(:, :, 1) = [a1' a2' a3' a4'];  % 四机控制变量

dt = 0.01;  % 采样时间
m = 2;  % 质量
kp_f = 4.7;  % 跟随者控制器比例系数
kd_f = 5;  % 跟随者控制器微分系数
k_l = 4.7;  % 领航者控制器比例系数
t = 0:dt:30;

%% 实时解算
Phi_dt = zeros(4);
Phi_dt(1, 3) = dt; Phi_dt(2, 4) = dt;
Phi = eye(4) + Phi_dt;
Gam = [dt^2/2 0; 0 dt^2/2; dt 0; 0 dt];
figure
hold on
n = length(t);
for k = 2:n
    % 四机编队刚性矩阵需满足5个距离约束
    % 即D(2, 1, k)=D(3, 1, k)=D(3, 2, k)=D(4, 1, k)=D(4, 2, k)=d_exp
    D(:, :, k) = distancemat(X(:, :, k-1));
    A(:, :, k) = anglemat(X(:, :, k-1));
    e_D = d_r - d_r * eye(size(D(:, :, k))) - D(:, :, k);  % 距离误差
    e_V = (e2_D - e_D)/dt;  % 速度误差（距离误差变化量）
    e2_D = e_D;  % 本步距离误差赋值给上一步
    
    a = a_ini + sin((k - 1)*dt/2);  % 目标角度
    for m = 1:4  % 速度误差
        ei_V(1, m) = X(3, m, k-1) - v_r * cos(a);
        ei_V(2, m) = X(4, m, k-1) - v_r * sin(a);    
    end
    
    u = (kp_f * e_D - kd_f * e_V) / m;    % 控制量
    Ux(:, :, k) = -u .* cos(A(:, :, k));  % 控制量分解x方向
    Uy(:, :, k) = -u .* sin(A(:, :, k));  % 控制量分解y方向
    
    U(1, 1, k) = -k_l * ei_V(1, 1);  % 领航者1 x速度跟随
    U(2, 1, k) = -k_l * ei_V(2, 1);  % 领航者1 y速度跟随
    
    % 5个距离约束
    U(1, 2, k) = Ux(2, 1, k) - kd_f * ei_V(1, 2);  % 跟随者2 x方向速度、领航者位置跟随
    U(2, 2, k) = Uy(2, 1, k) - kd_f * ei_V(2, 2);  % 跟随者2 y方向速度、领航者位置跟随
    U(1, 3, k) = Ux(3, 1, k) + Ux(3, 2, k) - kd_f * ei_V(1, 3);  % 跟随者3 x方向速度、领航者位置跟随
    U(2, 3, k) = Uy(3, 1, k) + Uy(3, 2, k) - kd_f * ei_V(2, 3);  % 跟随者3 y方向速度、领航者位置跟随
    U(1, 4, k) = Ux(4, 1, k) + Ux(4, 2, k) - kd_f * ei_V(1, 4);  % 跟随者4 x方向速度、领航者位置跟随
    U(2, 4, k) = Uy(4, 1, k) + Uy(4, 2, k) - kd_f * ei_V(2, 4);  % 跟随者4 y方向速度、领航者位置跟随
      
    % 状态方程
    % [x     [1 0 dt  0    [x         [dt^/2     0
    %  y    = 0 1  0 dt  *  y       +  0     dt^/2  * [ax
    % vx    = 0 0  1  0  * vx       +  dt        0  *  ay]
    % vy](k)  0 0  0  1]   vy](k-1)    0        dt]    
    for m = 1:4  % 状态方程(矩阵形式)
       X(:, m, k) =  Phi * X(:, m, k-1) + Gam * U(:, m, k);
    end
    
    % 记录
    D_rec(k, :) = [D(2, 1, k) D(3, 1, k) D(3, 2, k) ...
                   D(4, 1, k) D(4, 2, k) D(4, 3, k)];
    x1(k, :) = X(:, 1, k)';  x2(k, :) = X(:, 2, k)';
    x3(k, :) = X(:, 3, k)';  x4(k, :) = X(:, 4, k)';
    a1(k, :) = U(:, 1, k)';  a2(k, :) = U(:, 2, k)';
    a3(k, :) = U(:, 3, k)';  a4(k, :) = U(:, 4, k)';
        
    % 实时画图
%     plot(X(1, 1, k), X(2, 1, k), 'kx')  
%     plot(X(1, 2, k), X(2, 2, k), 'r.')
%     plot(X(1, 3, k), X(2, 3, k), 'g.')
%     plot(X(1, 4, k), X(2, 4, k), 'b.')
%     pause(0.001)
end
% xlabel('x/m','fontsize',12); ylabel('y/m','fontsize',12);
% legend({'领航者1','跟随者2', '跟随者3', '跟随者4'},'Location','best');

%% 根据记录数据画图
plot(x1(:, 1), x1(:, 2), 'k-.', 'LineWidth', 1)
plot(x2(:, 1), x2(:, 2), 'r-', 'LineWidth', 1)
plot(x3(:, 1), x3(:, 2), 'g-', 'LineWidth', 1)
plot(x4(:, 1), x4(:, 2), 'b-', 'LineWidth', 1)
grid on
xlabel('x/m','fontsize',12); ylabel('y/m','fontsize',12);
title('轨迹曲线')
legend({'领航者1','跟随者2','跟随者3','跟随者4'},'Location','best')

figure
plot(t, D_rec(:, 1:5), 'LineWidth', 1)
grid on
xlabel('t/s','fontsize',12); ylabel('d/m','fontsize',12);
title('距离变化曲线')
legend({'距离2-1','距离3-1','距离3-2','距离4-1','距离4-2'},'Location','best')

figure
hold on
plot(t, vecnorm(x1(:,3:4), 2, 2), 'LineWidth', 1)
plot(t, vecnorm(x2(:,3:4), 2, 2), 'LineWidth', 1)
plot(t, vecnorm(x3(:,3:4), 2, 2), 'LineWidth', 1)
plot(t, vecnorm(x4(:,3:4), 2, 2), 'LineWidth', 1)
grid on
xlabel('t/s','fontsize',12); ylabel('v/m/s','fontsize',12);
title('速度曲线')
legend({'领航者1','跟随者2','跟随者3','跟随者4'},'Location','best')

