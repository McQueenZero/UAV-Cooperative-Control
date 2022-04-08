function nn=plotBinHistogram(his,kt,kb,threshold,thresholdlow,nn)

mm=zeros(1,72);
n=length(his);
x1 = 1:n;
k1 = kt;   k2=kb;
hh=his;
for i=1:n
if (hh(i)>=threshold)
    mm(i)=1;
elseif (hh(n)<=thresholdlow)
    mm(i)=0;
    else 
        mm(i)=nn(i);
    
end
end

subplot(2,2,3:4)
hold off
bar(x1,mm);            %%%%%%%%没有加实际线
axis([0 80 0 3]); 
hold on
ylabel('H值');
xlabel('扇区');
title('二值极坐标直方图');

line([k1,k1],[0,2],'LineStyle','-.', 'color','r');
line([k2,k2],[0,2],'LineStyle','--', 'color','g');
legend('危险程度','目标方向','避障方向')
nn=mm;


subplot(2,2,1);
