clc
clear
close all
%% 初始化
% 领航者状态变量
x1 = [1 1 0 10*3.6];  % x, y, vx, vy
X(:, :, 1) = [x1'];

v_r = 180;  % 期望参考速度
% 航迹状态量
xw1 = [90/180*pi 0];  % theta, w
W(:, :, 1) = [xw1'];

a1 = [0 0];  % ax, ay
U(:, :, 1) = [a1'];  % 领航者控制变量

rx = 3:10;
ry = tand(90-75) * (rx-1) + 1;
rp = [rx' ry'];  % L1参考点
jj = 1;

dt = 0.01;  % 采样时间
kw_l = 2;  % 角速度控制比例系数
kv_l = 0.1;  % 速度控制比例系数
t = 0:dt:5;

%% 实时解算
Phi_dt = zeros(4);
Phi_dt(1, 3) = dt; Phi_dt(2, 4) = dt;
Phi = eye(4) + Phi_dt;
Gam = [dt^2/2 0; 0 dt^2/2; dt 0; 0 dt];
figure
hold on
n = length(t);
for k = 2:n 
    l1d = norm(X(1:2, 1, k-1)-rp(jj, :)');  % l1距离
    dtheta = atan((X(2, 1, k-1)-rp(jj, 2)) / (X(1, 1, k-1)-rp(jj, 1)));  % 目标点与无人机间的方向角
    eta = dtheta - W(1, 1, k-1);
    
    if (rp(jj,1)-X(1, 1, k-1)) < 0
        dtheta = dtheta + pi;
    elseif (rp(jj,2)-X(2, 1, k-1)) < 0
        dtheta = dtheta + pi*2;        
    end
    
    v = norm(X(3:4, 1, k-1));
    as = 2 * v^2 / l1d * sin(eta);  % 横向(向心)加速度    
    W(2, 1, k) = kw_l * as / v;  % 更新角速度
    W(1, 1, k) = W(1, 1, k-1) + W(2, 1, k) * dt;  % 更新航向角
    
    e_V = v - v_r;
    U(1, 1, k) = -kv_l * e_V * cos(W(1, 1, k));  % x向加速度
    U(2, 1, k) = -kv_l * e_V * sin(W(1, 1, k));  % y向加速度
    
    % 状态方程
    % [x     [1 0 dt  0    [x         [dt^/2     0
    %  y    = 0 1  0 dt  *  y       +  0     dt^/2  * [ax
    % vx    = 0 0  1  0  * vx       +  dt        0  *  ay]
    % vy](k)  0 0  0  1]   vy](k-1)    0        dt]    
    X(:, 1, k) =  Phi * X(:, 1, k-1) + Gam * U(:, 1, k);
    x1(k, :) = X(:, 1, k)';
    a1(k, :) = U(:, 1, k)';
        
    % 实时画图
    plot(X(1, 1, k), X(2, 1, k), 'k.')  
    pause(0.001)
    
    if l1d < 0.01  % 更新目标点
        if jj < length(rp)
            jj = jj + 1;
        else
            break  % 到达最后目标点后终止运行
        end
    end

end
plot(rp(:,1), rp(:,2), 'rx', 'LineWidth', 1)
legend({'领航者1', 'L1导航点'},'Location','best');

%% 根据记录数据画图
% figure
% hold on
% plot(x1(:, 1), x1(:, 2), 'k-.', 'LineWidth', 1)
% plot(rp(:,1), rp(:,2), 'rx', 'LineWidth', 1)
% grid on
% xlabel('x/km','fontsize',12); ylabel('y/km','fontsize',12);
% title('轨迹曲线')
% legend({'领航者1'},'Location','best')
