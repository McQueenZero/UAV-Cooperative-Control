%%��������ֱ��ͼ

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
bar(x1,his);            %%%%%%%%û�м�ʵ����
hold on
ylabel('Hֵ');
xlabel('����');
title('��������ֱ��ͼ');

line([k1,k1],[0,max(his)],'LineStyle','-.', 'color','r');%line([X1 X2],[Y1 Y2],S);
line([k2,k2],[0,max(his)],'LineStyle','--', 'color','g');

line([0,n],[threshold,threshold],'LineStyle','-', 'color','k');
line([0,n],[thresholdlow,thresholdlow],'LineStyle','-', 'color','y');
legend('Σ�ճ̶�','Ŀ�귽��','���Ϸ���','����ֵ','����ֵ')

 %%ֱ��ͼ�ͱ���ͼ�������
subplot(2,2,3:4);