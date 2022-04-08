ob = imread('picture1.png');
s = ob;
s = double(s);
[row, column] = size(ob);
n = 1;
obstacle = zeros(2000, 2);

for i = 1:row
    for j = 1:column
        if s(i, j) == 0
            obstacle(n, 1:2) = [i, j];
            n = n + 1;
        end
    end
end

obstacle = obstacle / 50;
save ('obstacle.mat', 'obstacle');
