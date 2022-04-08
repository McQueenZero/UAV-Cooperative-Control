%%画极坐标直方图

function plotHistogram(his,kt,kb,threshold,thresholdlow)
%global his kt kb


n=length(his);
x1 = [1:n];
k1 = kt;   k2=kb;

y = [0:max(his)];
if(max(his) <=1)
    y=[0:0.01:1]; %to get a smoother line
end
subplot(2,2,2);
hold off
bar(x1,his);            %%%%%%%%没有加实际线
hold on
ylabel('H值');
xlabel('扇区');
title('主极坐标直方图');

line([k1,k1],[0,max(his)],'LineStyle','-.', 'color','r');%line([X1 X2],[Y1 Y2],S);
line([k2,k2],[0,max(his)],'LineStyle','--', 'color','g');

line([0,n],[threshold,threshold],'LineStyle','-', 'color','k');
line([0,n],[thresholdlow,thresholdlow],'LineStyle','-', 'color','y');
legend('危险程度','目标方向','避障方向','高阈值','低阈值')

 %%直方图和避障图交替进行
subplot(2,2,3:4);