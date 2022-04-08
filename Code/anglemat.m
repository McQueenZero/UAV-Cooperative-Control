function A = anglemat(X)
%输入状态矩阵计算角度邻接矩阵
%   状态矩阵的形式为X=
%   [x1, x2, ...]
%   其中x1 = [x; y; vx; vy] 即x坐标, y坐标, x向速度, y向速度
%   A(a, b) 表示xa指向xb的向量辐角

A = zeros(size(X, 2));
for ii = 1:size(X,2)
    for jj = 1:size(X,2)
        p1 = X(1:2, ii);
        p2 = X(1:2, jj);
        A(jj, ii) = atan2(p1(2)-p2(2), p1(1)-p2(1));
%         A(jj, ii) = atan2d(p1(2)-p2(2), p1(1)-p2(1));
    end
end
end