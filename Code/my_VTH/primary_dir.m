function [kt, current_point, forward_direction] = primary_dir(obstacle_density, ...
    kt, current_point, forward_direction, projected_point, endpoint, ...
    thresholdhigh, smax, n, alpha, step)
j = 1;
q = 1;
while q <= n
    if obstacle_density(q) < thresholdhigh
        kr = q;  % 当前步的障碍物密度
        while (q <= n) && (obstacle_density(q) < thresholdhigh)
            kl = q;
            q = q + 1;
        end
        if (kl - kr) > smax
            Alternative_direction(j) = round(kl - smax / 2);
            Alternative_direction(j+1) = round(kr + smax / 2);
            j = j + 2;  % (VFH+, 13,14)
            if (kt >= kr) && (kt <= kl)
                Alternative_direction(j) = kt;
                j = j + 1;
            end
        elseif (kl - kr) > (smax / 5)
            Alternative_direction(j) = round((kr + kl) / 2 - 2.5);
            j = j + 1;
        end
    else
        q = q + 1;
    end
end
%合适的方向存储在Alternative_direction里面，下面找到最优%
num1 = zeros(j - 1, 1);
num2 = zeros(j - 1, 1);
for i = 1 : j - 1
    num1(i) = Alternative_direction(i);  % 主要候选方向
    num2(i) = 5 * caculate_abs(num1(i), kt) + ...  % (VFH*, 2)
        2 * caculate_abs(num1(i), caculate_beta(projected_point,endpoint)/alpha) + ...
        2 * caculate_abs(num1(i), forward_direction);
end
min_num = find(num2 == min(num2));
min_num_value = min_num(1);
forward_direction = num1(min_num_value);
current_point = current_point + [step * cos(forward_direction * alpha), step * sin(forward_direction * alpha)];

kt=round(caculate_beta(current_point,endpoint)/alpha);
if(kt==0)
    kt=n;
end

end