clear all
R=100;

v(1,:) = 0.5; %线速度
theta(1,1) = pi/2;%航向
p(1,:) = [1,1];%位置

w(1,1) = 0;%角速度

cx=3:50;
cy=0.466*cx+0.534;
dp=[cx',cy',ones(48,1)];
dt = 0.01;%
n = 11000;%仿真步长
j = 1;

Lfc = 0;
k = 1;
n_t = length(dp);

for i = 2:n
    l1d = sqrt((dp(j,2)-p(i-1,2))^2+(dp(j,1)-p(i-1,1))^2); %L1距离
    dtheta = atan((dp(j,2)-p(i-1,2))/(dp(j,1)-p(i-1,1))); %目标点与无人机间的方向角：目标角
    if (dp(j,1)-p(i-1,1)) < 0
        dtheta = dtheta + pi;
    elseif (dp(j,2)-p(i-1,2)) < 0
        dtheta = dtheta + pi*2;        
    end
    eta = dtheta - theta(i-1,1); %航向角与目标角差值
     
    a_y = 2 * v(i-1,1)^2/l1d*sin(eta); %横向加速度
    w(i,1) = 2*a_y/v(i-1,1); %更新角速度
    v(i,1) = v(i-1,1);%更新线速度
    
    theta(i,1) = theta(i-1,1) + w(i,1)*dt; %更新航向角
    p(i,1) = p(i-1,1) + v(i,1)*dt * cos(theta(i,1));%更新位置
    p(i,2) = p(i-1,2) + v(i,1)*dt * sin(theta(i,1));
   if l1d < 0.01%更新目标点
        if j <n_t
            j = j + 1;
        else
            break;%到达最后目标点后终止运行
        end
    end

end

figure(1)
plot(p(:,1),p(:,2),'bo')
hold on
plot(dp(:,1),dp(:,2),'rx')
hold off
grid on
n = length(p(:,1));
t = 0:dt:(n-1)*dt;
% subplot(212),plot(t,theta)
