function D = distancemat(X)
%输入状态矩阵计算距离邻接矩阵
%   状态矩阵的形式为X=
%   [x1, x2, ...]
%   其中x1 = [x; y; vx; vy] 即x坐标, y坐标, x向速度, y向速度
    
D = zeros(size(X, 2));
for ii = 1:size(X,2)
    for jj = 1:size(X,2)
        p1 = X(1:2, ii);
        p2 = X(1:2, jj);
        D(ii, jj) = norm(p2 - p1);
    end
end
end