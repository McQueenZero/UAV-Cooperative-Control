clear
%对领航者的初始化
pl(1,:) = [1,1];
vl(1,1) = 0;
vlx(1,1) = 0;
vly(1,1)=0;
thetal(1,1)=pi/2;
vL_r = 50;
thetal_r= 15/180*pi;
n=800;

kp1 = 4.7;
kp2 = 4.7;
kp_w = 4.7;
kd_w = 5;
kp3 = 4.7;
dt = 0.01;%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%首先领航者进行领航

for m = 2:n
    elv = vL_r - vl(m-1,1);
    ethetal = thetal_r - thetal(m-1,1);
    
    uvl(m,1)=kp1 * elv ;
    utl(m,1)=kp2 * ethetal;
    vl(m,1) = vl(m-1,1) + uvl(m,1)*dt;
    thetal(m,1) =   thetal(m-1,1) + utl(m,1)*dt;
    vlx(m,1) =   vl(m,1)*cos(thetal(m,1));
    vly(m,1) =   vl(m,1)*sin(thetal(m,1));
    pl(m,1) = pl(m-1,1) + 1/2*(vlx(m,1) + vlx(m-1,1)) * dt;
    pl(m,2) = pl(m-1,2) + 1/2*(vly(m,1) + vly(m-1,1)) * dt;
    plot(pl(m,1), pl(m,2),'go')
    hold on;
    if m>400
        if m<410
        dp(m-400,1) = pl(m,1)-50*cos(pi/12);
        dp(m-400,2) = pl(m,2)+50*sin(pi/12);
        end
    end
end
 %采用速度控制与航向控制
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%完成跟随者的L1路径跟随
                                                                                                                                                                                                                                                                                                                            

vf(1,1) = 2; %线速度
thetaf(1,1) = pi/2;%航向
pf(1,:) = [0,0];%位置
d_d = 50;
dfl(1,1) = sqrt((pl(1,1)-pf(1,1))^2+(pl(1,2)-pf(1,2))^2);%位置控制
e_d1fl = d_d - dfl(1,1);
e_d2fl = e_d1fl;
w(1,1) = 0;%角速度


n = 11000;%仿真步长
j = 1;

Lfc = 0;
k = 1;
n_t = length(dp);

for i = 2:n
    l1d = sqrt((dp(j,2)-pf(i-1,2))^2+(dp(j,1)-pf(i-1,1))^2); %L1距离
    dtheta = atan((dp(j,2)-pf(i-1,2))/(dp(j,1)-pf(i-1,1))); %目标点与无人机间的方向角：目标角
    if (dp(j,1)-pf(i-1,1)) < 0
        dtheta = dtheta + pi;
    elseif (dp(j,2)-pf(i-1,2)) < 0
        dtheta = dtheta + pi*2;        
    end
    eta = dtheta - thetaf(i-1,1); %航向角与目标角差值
     
    a_y = 2 * vf(i-1,1)^2/l1d*sin(eta); %横向加速度
    w(i,1) = 2*a_y/vf(i-1,1); %更新角速度
    vf(i,1) = vf(i-1,1);%更新线速度
    
    thetaf(i,1) = thetaf(i-1,1) + w(i,1)*dt; %更新航向角
    pf(i,1) = pf(i-1,1) + vf(i,1)*dt * cos(thetaf(i,1));%更新位置
    pf(i,2) = pf(i-1,2) + vf(i,1)*dt * sin(thetaf(i,1));
    
   if l1d < 0.01%更新目标点
        if j <n_t
            j = j + 1;
        else
            break;%到达最后目标点后终止运行
        end
    end

end

figure(1)
plot(pf(:,1),pf(:,2))
hold on
plot(dp(:,1),dp(:,2),'rx')
hold off
grid on
n = length(pf(:,1));
t = 0:dt:(n-1)*dt;


%更新领航者的位置
a=i;
pl(1,1) = pl(m,1) ;
pl(1,2) = pl(m,2) ;
vlx(1,1) = vlx(m,1);
vly(1,1)= vly(m,1);
pf(1,1) =pf(a,1);
pf(1,2) =pf(a,2);
vf(1,1)=0;
n=2000;
dfl(1,1) = sqrt((pl(1,1)-pf(1,1))^2+(pl(1,2)-pf(1,2))^2);
e_d1fl = d_d - dfl(1,1);
e_d2fl = e_d1fl;

for i = 2:n
    pl(i,1) = pl(i-1,1) + vlx(1,1)*dt;
    pl(i,2) = pl(i-1,2) + vly(1,1)*dt;
    
    dfl(i,1) = sqrt((pl(i-1,1)-pf(i-1,1))^2+(pl(i-1,2)-pf(i-1,2))^2);
    e_d1fl = d_d - dfl(i,1);
    ev_dfl = (e_d2fl - e_d1fl)/dt;
    e_d2fl = e_d1fl;
    d(i,1)= -e_d1fl;
    u_w(i,1)  = kp_w *  e_d1fl  - kd_w * ev_dfl ;
    
    elvf = vL_r - vf(i-1,1);
    uvf(i,1)=kp1 * elvf ;
    
    
    
    vf(i,1) = vf(i-1,1) + (uvf(i,1)-u_w(i,1))*dt;
    
    vfx(i,1) =   vf(i,1)*cos(thetaf(a,1));
    vfy(i,1) =   vf(i,1)*sin(thetaf(a,1));
    pf(i,1) = pf(i-1,1) + 1/2*(vfx(i,1) + vfx(i-1,1)) * dt;
    pf(i,2) = pf(i-1,2) + 1/2*(vfy(i,1) + vfy(i-1,1)) * dt;
    jiaodu(i,1) =  (atan2(pf(i,2)-pl(i,2),pl(i,1)-pf(i,1))+thetal(m))/pi*180;
    
end

figure(2);
subplot(311),plot(dfl);
subplot(312),plot(d);
subplot(313),plot(jiaodu);
t = 0:dt:(n-1)*dt;




%%%%%%%%%%%%%%%%%%%%%%%%
%图一表示L1跟随过程，
%图二第一个图表示跟随者与领航者之间的距离，第一个图表示与目标距离的差值，第三个图是跟随者与领航者之间的角度



