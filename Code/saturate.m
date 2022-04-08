function x = saturate(x, low, high)
if low < high  
    if x < low
        x = low;
    elseif x > high
        x = high;
    end
end
end