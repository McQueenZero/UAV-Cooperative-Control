%计算绝对值差的最小值%  (VFH+, 17)
function dif = caculate_abs(c1, c2)
n = 72;
dif = min([abs(c1 - c2), abs(c1 - c2 - n), abs(c1 - c2 + n)]);