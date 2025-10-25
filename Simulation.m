clear all; close all; clc;
global OK;
global vehsD vehstate t;
global palya;
global va;
global costmap kormanyszog sebesseg;

%vehicle parameters
C1 = 80000;
C2 = 120000;
l1 = 2.2;
l2 = 2.3;
J = 2500;
m = 1500;
Ts=0.1;
%x_dot = A * [ddot_psi; dot_vy; vy] + B * delta;


va=60/3.6;
vh=40/3.6;          %human-driven vehicle sebessége
Ts=0.1;
vehsD=[0;0;0];          %initial condition
vehstate(1,:)=[1, 2.5, 0, 15];  %initial condition
kormanyszog=0;
sebesseg(1)=va;
T=[0:Ts:15*Ts];


for t=2:length(T)
    %megtervezzük a pályát
    palya=palyagen(vehstate(t-1,:));
    %
    %jmumozgas a human-driven jmure
    vehstate(t,4)=vehstate(t-1,4)-Ts*vh;
    %
    %mozgas optimalizálása az autonom jmure
    A=[];
    b=[];
    Aeq=[];
    beq=[];
    lb=-15*pi/180;
    ub=15*pi/180;
    [delta,FVAL,EXITFLAG] = fmincon(@fun,0,A,b,Aeq,beq,lb,ub);
    korm=0.8*delta+0.2*kormanyszog(t-1);
    %kiszámítjuk a végleges értékeket
    %kiszámítjuk a végleges értékeket
    if palya(1,:)==palya(2,:)
        disp('nem lepett be')
        seb=max(round(sebesseg(t-1)*3.6)-3,0.01);
    else;
        disp('belepett')
        seb=sebopt(korm);
    end;
    v_x = seb/3.6;
    A = [(-C1*l1^2-C2*l2^2)/(J*v_x), (-C1*l1+C2*l2)/(J*v_x), 0;
         (-C1*l1+C2*l2)/(m*v_x)-v_x, (-C1-C2)/(m*v_x), 0;
         0, 1, 0];
    B = [C1*l1/J; C1/m; 0];
    C=[0 0 1;
        1 0 0];
    D=[0;0];
    sysC=ss(A,B,C,D);
    sysD=c2d(sysC,Ts);
    vehsD=sysD.A*vehsD+sysD.B*korm;
    vehstate(t,3)=vehsD(1);
    vehstate(t,1)=vehstate(t-1,1)+v_x*Ts*cos(vehstate(t,3)); %[1, 2.5, 0, 15];
    vehstate(t,2)=vehstate(t-1,2)+v_x*Ts*sin(vehstate(t,3));
    fin=vehstate(t,1:3)
    sebesseg(t)=v_x;
    kormanyszog(t)=korm;
end;
save results;


%plottolás
load results;
for i=1:length(vehstate)
    if vehstate(i,1)<25
        figure;
        palyagen(vehstate(i,:));
    end;
end;

figure;
plot([0:Ts:15*Ts],[va sebesseg(2:length(sebesseg))]*3.6)

figure;
plot([0:Ts:15*Ts],[0 kormanyszog(2:length(kormanyszog))]*180/pi)


