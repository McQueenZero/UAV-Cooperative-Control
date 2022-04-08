function [kt, projected_point, forward_direction] = projected_dir(obstacle_density, ...
    kt, current_point, cim1_point, endpoint, ...
    thresholdhigh, smax, n, alpha, step, ng)
step_pjt = step * ng;
j = 1;
q = 1;
while q <= n
    if obstacle_density(q) < thresholdhigh
        kr = q;
        while (q <= n) && (obstacle_density(q) < thresholdhigh)
            kl = q;
            q = q + 1;
        end
        if (kl - kr) > smax
            Alternative_direction(j) = round(kl - smax / 2);
            Alternative_direction(j+1) = round(kr + smax / 2);
            j = j + 2;
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
    num1(i) = Alternative_direction(i);  % 投影方向
    num2(i) = 0.8 * (5 * max(caculate_abs(num1(i), kt), caculate_abs(caculate_beta(cim1_point,current_point), kt)) + ...
        1 * caculate_abs(num1(i), caculate_beta(current_point,endpoint)/alpha) + ...
        1 * caculate_abs(num1(i), cim1_point));
end
min_num = find(num2 == min(num2));
min_num_value = min_num(1);
forward_direction = num1(min_num_value);
projected_point = current_point + [step_pjt * cos(forward_direction * alpha), step_pjt * sin(forward_direction * alpha)];

kt=round(caculate_beta(projected_point,endpoint)/alpha);
if(kt==0)
    kt=n;
end

end