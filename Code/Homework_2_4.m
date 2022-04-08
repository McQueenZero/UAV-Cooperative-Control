%%-------------------------------------------------------------------------
% 作者：   赵敏琨
% 学号：   2018302068
% 时间：   2021年12月
%%-------------------------------------------------------------------------
clc
clear
close all
%% 初始化
% 两机状态变量
x1 = [1000 1000 20];  % x, y, v
x2 = [0 0 20];
X(:, :, 1) = [x1' x2'];

dis = norm([1000 1000]);  % 两机距离
btheta = 45/180*pi;  % 两机角度

v_r = 180 / 3.6;  % 初始期望参考速度
% 航迹状态量
xw1 = [90/180*pi 0];  % theta, w
xw2 = xw1;
W(:, :, 1) = [xw1' xw2'];

% 两机控制变量
a1 = [0 0]; a2 = [0 0];  % alon, alat 
U(:, :, 1) = [a1' a2'];  

rx = 1400:300:8600;
ry = tand(90-75) * (rx-1000) + 1000;
rp = [rx' ry'];  % 领航者L1参考点
rp_f = [1000 1000];  % 跟随者L1参考点初始化
drp_f = 600;  % 跟随者L1导航点间隔
jj = 1;

dt = 0.01;  % 采样时间
kw = 2;  % 角速度控制比例系数
kv = 0.3;  % 速度控制比例系数
kp = 0.03;  % 位置控制比例系数
t = 0:dt:150;

%% 实时解算
figure
hold on
n = length(t);
plot(rp(:,1), rp(:,2), 'kx', 'MarkerSize', 6, 'LineWidth', 1)
for k = 2:n 
    l1d_l = norm(X(1:2, 1, k-1)-rp(jj, :)');  % 领航者L1距离
    dtheta_l = atan((X(2, 1, k-1)-rp(jj, 2)) / (X(1, 1, k-1)-rp(jj, 1)));  % 目标点与领航者间的方向角
    eta_l = dtheta_l - W(1, 1, k-1);    
    if (rp(jj,1)-X(1, 1, k-1)) < 0
        dtheta_l = dtheta_l + pi;
    elseif (rp(jj,2)-X(2, 1, k-1)) < 0
        dtheta_l = dtheta_l + pi*2;        
    end
    
    % 计算跟随者的L1参考点
    rx_f = X(1, 1, k-1) - 500*cosd(30)*cos(W(1, 1, k-1)) - 500*sind(30)*cosd(60);
    ry_f = X(2, 1, k-1) - 500*cosd(30)*sin(W(1, 1, k-1)) + 500*sind(30)*sind(60);
    if mod(k+1, drp_f) == 0 
        rp_f = [rp_f; rx_f ry_f];
    end
    
    l1d_f = norm(X(1:2, 2, k-1)-rp_f(end, :)');  % 跟随者L1距离
    dtheta_f = atan((X(2, 2, k-1)-rp_f(end, 2)) / (X(1, 2, k-1)-rp_f(end, 1)));  % 目标点与跟随者间的方向角
    eta_f = dtheta_f - W(1, 2, k-1);    
    if (rp_f(end,1)-X(1, 2, k-1)) < 0
        dtheta_f = dtheta_f + pi;
    elseif (rp_f(end,2)-X(2, 2, k-1)) < 0
        dtheta_f = dtheta_f + pi*2;        
    end
    
    e_V = X(3, 1, k-1) - v_r;
    U(1, 1, k) = -kv * e_V;  % 纵向加速度
    U(2, 1, k) = 2 * X(3, 1, k-1)^2 / l1d_l * sin(eta_l);  % 横向加速度
    
    W(2, 1, k) = kw * U(2, 1, k) / X(3, 1, k-1);  % 更新角速度
    W(1, 1, k) = W(1, 1, k-1) + W(2, 1, k) * dt;  % 更新航向角
    
    e_P = (X(1, 2, k-1) - rp_f(end, 1))/cos(W(1, 2, k-1));
    e_D = dis(k-1, 1)*cos(btheta(k-1, 1)) - 500*cosd(30); 
    % 距离指领航者yoz平面与跟随者的距离
    if abs(e_D) > 100  % 距离>100m则轨迹跟踪，反馈速度控制
        e_V = X(3, 2, k-1) - (X(3, 1, k-1) + 40/3.6);
        U(1, 2, k) = -kv * e_V;  % 纵向加速度
        U(2, 2, k) = 2 * X(3, 2, k-1)^2 / l1d_f * sin(eta_f);  % 横向加速度
        W(2, 2, k) = kw * U(2, 2, k) / X(3, 2, k-1);  % 更新角速度
        W(1, 2, k) = W(1, 2, k-1) + W(2, 2, k) * dt;  % 更新航向角 
    else % 否则速度减小，反馈速度与距离控制
        e_V = X(3, 2, k-1) - X(3, 1, k-1);
        U(1, 2, k) = -kv * e_V + kp * e_D;  % 纵向加速度 
        U(2, 2, k) = U(2, 2, k);  % 横向加速度
        W(2, 2, k) = W(2, 2, k);  % 更新角速度
        W(1, 2, k) = W(1, 2, k-1) + W(2, 2, k) * dt;  % 更新航向角   
    end
            
    for m = 1:2
        Phi_dp = [0 0 dt*cos(W(1, m, k))
                  0 0 dt*sin(W(1, m, k))
                  0 0                 0 ];
        Phi = eye(3) + Phi_dp;
        Gam(:, :, m) = [0 0; 0 0; dt 0];
        
        % 状态方程
        % [x       [1 0 dt*cos(theta)    [x         [0  0    [alon
        %  y     =  0 1 dt*sin(theta)  *  y       +  0  0  *  alat] 
        %  v](k)    0 0            1 ]    v](k-1)    dt 0]
        X(:, m, k) =  Phi * X(:, m, k-1) + Gam(:, :, m) * U(:, m, k-1);
    end
    x1(k, :) = X(:, 1, k)'; x2(k, :) = X(:, 2, k)';
    a1(k, :) = U(:, 1, k)'; a2(k, :) = U(:, 2, k)';
    xw1(k, :) = W(:, 1, k)'; xw2(k, :) = W(:, 2, k)';
    dis(k, 1) = norm(x1(k, :)-x2(k, :));
    btheta(k, 1) = -atan2(x1(k,2)-x2(k,2),x1(k,1)-x2(k,1)) + W(1, 1, k);
    
    % 实时画图
%     plot(X(1, 1, k), X(2, 1, k), 'r.')  
%     plot(X(1, 2, k), X(2, 2, k), 'b.') 
%     if mod(k+1, drp_f) == 0 
%         plot(rp_f(end, 1), rp_f(end, 2), 'gx', 'LineWidth', 1)
%     end
%     pause(0.001)
    
    if l1d_l < 1  % 更新目标点
        if jj < length(rp)
            jj = jj + 1;
        else
%             break % 到达最后目标点后退出
        end
    end    
end
% xlabel('x/m','fontsize',12); ylabel('y/m','fontsize',12);
% legend({'领航者L1导航点', '领航者1', '跟随者2', '跟随者L1导航点'},'Location','best');

%% 根据记录数据画图
plot(x1(:, 1), x1(:, 2), 'r-', 'LineWidth', 1)
plot(x2(:, 1), x2(:, 2), 'b-', 'LineWidth', 1)
plot(rp_f(:, 1), rp_f(:, 2), 'gx', 'LineWidth', 1)
grid on
xlabel('x/m','fontsize',12); ylabel('y/m','fontsize',12);
title('轨迹曲线')
legend({'领航者L1导航点', '领航者1', '跟随者2', '跟随者L1导航点'},'Location','best')

figure
hold on
plot(t, rad2deg(xw1(:, 1)), 'LineWidth', 1)
plot(t, rad2deg(xw1(:, 2)), 'LineWidth', 1)
grid on
xlabel('t/s','fontsize',12); ylabel('\theta/deg \omega/deg/s','fontsize',12);
title('领航者航向角/角速度曲线')
legend({'航向角', '角速度'},'Location','best')

figure
subplot(2,2,1)
hold on
plot(t, x1(:, 3), 'LineWidth', 1)
plot(t, x2(:, 3), 'LineWidth', 1)
grid on
xlabel('t/s','fontsize',12); ylabel('v/m/s','fontsize',12);
title('速度曲线')
legend({'领航者1', '跟随者2'},'Location','best')

subplot(2,2,2)
hold on
plot(t, rad2deg(xw2(:, 1)), 'LineWidth', 1)
plot(t, rad2deg(xw2(:, 2)), 'LineWidth', 1)
grid on
xlabel('t/s','fontsize',12); ylabel('\theta/deg \omega/deg/s','fontsize',12);
title('跟随者航向角/角速度曲线')
legend({'航向角', '角速度'},'Location','best')

subplot(2,2,3)
plot(t, dis, 'LineWidth', 1)
grid on
xlabel('t/s','fontsize',12); ylabel('d/m','fontsize',12);
title('领航者跟随者距离曲线')

subplot(2,2,4)
plot(t, rad2deg(btheta), 'LineWidth', 1)
grid on
xlabel('t/s','fontsize',12); ylabel('btheta/deg','fontsize',12);
title('领航者跟随者角度曲线')
