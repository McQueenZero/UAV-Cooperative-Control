%% VFH+、*算法
% 运行VFH+算法注释掉63~64、85~91行
% 运行VFH*算法注释掉62、78~80行
clc
clear
close all
load obstacle 'obstacle';
startpoint = [0 0];
endpoint = [6.5 9.5];
% endpoint = [6 9];
%障碍路径图%
subplot(2, 2, 1);
plot(obstacle(:, 1), obstacle(:, 2), '.k');
hold on
plot(startpoint(:, 1), startpoint(:, 2), '.b');
hold on
plot(endpoint(:, 1), endpoint(:, 2), '.r');
hold on
title('Implementation of algorithm');
%VFH 算法变量定义%
step_rec = 0;
step = 0.1;                 %机器人步进值
f = 5;                      %角分辨率， 单位°
dmax = 1;                   %激光雷达检测长度
smax = 18;                  %宽波谷窄波谷阈值
b = 2.5;                    %常量
a = 1 + b.*(dmax.^2);       %常量
C = 15;                     %cv初始值
alpha = deg2rad(f);         %角分辨率， 单位弧度
n = 360 / f;                %扇区数量
thresholdhigh = 3000;       %二值化直方图高阈值
thresholdlow = 2000;        %二值化直方图低阈值
rsafe = 0.5;                %机器人安全距离
current_point = startpoint; %机器人实时位置
%定义机器人目标方向%
kt = round(caculate_beta(current_point, endpoint) / alpha);
if kt == 0
    kt = n;
end
%定义机器人初始避障方向%
forward_direction = kt;
cim1_point = [0 0];
%定义二元直方图上次值%
binary_hisvalue = zeros(1, 72);
%算法实现，步骤为H值-》安全角-》机器人下一坐标%
%首先计算每一个扇区H值%
while norm(current_point - endpoint) ~= 0
    if norm(current_point - endpoint) > step
        i = 1;
        obstacle_amplitude = zeros(n, 1);
        obstacle_density = zeros(n, 1);
        while i<= length(obstacle)
            obstacle_distance = norm(obstacle(i, : ) - current_point);
            if obstacle_distance < dmax
                beta = caculate_beta(current_point, obstacle(i, : ));
                enlarged_ange = asin(rsafe / obstacle_distance);  % 安全角
                k = round(beta / alpha);  % 当前方向
                if k == 0
                    k = n;
                end
                if((5*k>rad2deg(beta)-rad2deg(enlarged_ange))&&(5*k<rad2deg(beta)+rad2deg(enlarged_ange)))
%                     h(k) = 1;  % (VFH+, 5,6)
                    h(k) = 1 * caculate_abs(k, caculate_beta(current_point,endpoint)/alpha) + ...
                        1 * caculate_abs(k, cim1_point);  % (VFH*, 8)
                else
                    h(k) = 0;
                end
                m = C^2 * (a-b*(obstacle_distance.^2));  % (VFH+, 2)
                obstacle_amplitude(k) = obstacle_amplitude(k) + m.*h(k);
                i = i + 1;
            else
                i = i + 1;
            end
        end
        %得到扇区密度值%
        obstacle_density = obstacle_amplitude;
        %VFH+：下面函数计算出目标向量kt，最佳前进方向angle，机器人下一坐标robot%
%         [kt, current_point, forward_direction] = primary_dir(obstacle_density, ...
%             kt, current_point, forward_direction, current_point, endpoint, ...
%             thresholdhigh, smax, n, alpha, step);

        %VFH*：下面投影候选方向函数计算出投影点和最佳前进方向angle%
        %VFH*：下面主要候选方向函数计算出目标向量kt，机器人下一坐标robot，
        %      并更新前进方向%
        [~, projected_point, forward_direction] = projected_dir(obstacle_density, ...
            kt, current_point, cim1_point, endpoint, ...
            thresholdhigh, smax, n, alpha, step, 5);  % ng       
        [kt, current_point, forward_direction] = primary_dir(obstacle_density, ...
            kt, current_point, forward_direction, projected_point, endpoint, ...
            thresholdhigh, smax, n, alpha, step);
        cim1_point = current_point;  % c_{i-1}
        
        scatter(current_point(1), current_point(2), '.m');
        drawnow;
        %画极坐标直方图%
        plotHistogram(obstacle_density,kt,forward_direction,thresholdhigh,thresholdlow);
        binary_hisvalue = plotBinHistogram(obstacle_density,kt,forward_direction,thresholdhigh,thresholdlow,binary_hisvalue);
        step_rec = step + step_rec;
    else
        disp(['路径长度：' num2str(step_rec)])
        break
    end
end





