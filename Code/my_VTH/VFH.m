%% VFH+��*�㷨
% ����VFH+�㷨ע�͵�63~64��85~91��
% ����VFH*�㷨ע�͵�62��78~80��
clc
clear
close all
load obstacle 'obstacle';
startpoint = [0 0];
endpoint = [6.5 9.5];
% endpoint = [6 9];
%�ϰ�·��ͼ%
subplot(2, 2, 1);
plot(obstacle(:, 1), obstacle(:, 2), '.k');
hold on
plot(startpoint(:, 1), startpoint(:, 2), '.b');
hold on
plot(endpoint(:, 1), endpoint(:, 2), '.r');
hold on
title('Implementation of algorithm');
%VFH �㷨��������%
step_rec = 0;
step = 0.1;                 %�����˲���ֵ
f = 5;                      %�Ƿֱ��ʣ� ��λ��
dmax = 1;                   %�����״��ⳤ��
smax = 18;                  %����խ������ֵ
b = 2.5;                    %����
a = 1 + b.*(dmax.^2);       %����
C = 15;                     %cv��ʼֵ
alpha = deg2rad(f);         %�Ƿֱ��ʣ� ��λ����
n = 360 / f;                %��������
thresholdhigh = 3000;       %��ֵ��ֱ��ͼ����ֵ
thresholdlow = 2000;        %��ֵ��ֱ��ͼ����ֵ
rsafe = 0.5;                %�����˰�ȫ����
current_point = startpoint; %������ʵʱλ��
%���������Ŀ�귽��%
kt = round(caculate_beta(current_point, endpoint) / alpha);
if kt == 0
    kt = n;
end
%��������˳�ʼ���Ϸ���%
forward_direction = kt;
cim1_point = [0 0];
%�����Ԫֱ��ͼ�ϴ�ֵ%
binary_hisvalue = zeros(1, 72);
%�㷨ʵ�֣�����ΪHֵ-����ȫ��-����������һ����%
%���ȼ���ÿһ������Hֵ%
while norm(current_point - endpoint) ~= 0
    if norm(current_point - endpoint) > step
        i = 1;
        obstacle_amplitude = zeros(n, 1);
        obstacle_density = zeros(n, 1);
        while i<= length(obstacle)
            obstacle_distance = norm(obstacle(i, : ) - current_point);
            if obstacle_distance < dmax
                beta = caculate_beta(current_point, obstacle(i, : ));
                enlarged_ange = asin(rsafe / obstacle_distance);  % ��ȫ��
                k = round(beta / alpha);  % ��ǰ����
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
        %�õ������ܶ�ֵ%
        obstacle_density = obstacle_amplitude;
        %VFH+�����溯�������Ŀ������kt�����ǰ������angle����������һ����robot%
%         [kt, current_point, forward_direction] = primary_dir(obstacle_density, ...
%             kt, current_point, forward_direction, current_point, endpoint, ...
%             thresholdhigh, smax, n, alpha, step);

        %VFH*������ͶӰ��ѡ�����������ͶӰ������ǰ������angle%
        %VFH*��������Ҫ��ѡ�����������Ŀ������kt����������һ����robot��
        %      ������ǰ������%
        [~, projected_point, forward_direction] = projected_dir(obstacle_density, ...
            kt, current_point, cim1_point, endpoint, ...
            thresholdhigh, smax, n, alpha, step, 5);  % ng       
        [kt, current_point, forward_direction] = primary_dir(obstacle_density, ...
            kt, current_point, forward_direction, projected_point, endpoint, ...
            thresholdhigh, smax, n, alpha, step);
        cim1_point = current_point;  % c_{i-1}
        
        scatter(current_point(1), current_point(2), '.m');
        drawnow;
        %��������ֱ��ͼ%
        plotHistogram(obstacle_density,kt,forward_direction,thresholdhigh,thresholdlow);
        binary_hisvalue = plotBinHistogram(obstacle_density,kt,forward_direction,thresholdhigh,thresholdlow,binary_hisvalue);
        step_rec = step + step_rec;
    else
        disp(['·�����ȣ�' num2str(step_rec)])
        break
    end
end





