function kimenet=fun(delta)
global vehsD vehstate t;
global palya;
global va_max kormanyszog;
global sebesseg;

%vehicle parameters
C1 = 80000;
C2 = 120000;
l1 = 2.2;
l2 = 2.3;
J = 2500;
m = 1500;
Ts=0.1;

korm=0.8*delta+0.2*kormanyszog(t-1);

v_x = sebesseg(t-1);
A = [(-C1*l1^2-C2*l2^2)/(J*v_x), (-C1*l1+C2*l2)/(J*v_x), 0;
     (-C1*l1+C2*l2)/(m*v_x)-v_x, (-C1-C2)/(m*v_x), 0;
     0, 1, 0];
B = [C1*l1/J; C1/m; 0];
C=[0 0 1;
    1 0 0];
D=[0;0];
sysC=ss(A,B,C,D);
sysD=c2d(sysC,Ts);
vehsD_=sysD.A*vehsD+sysD.B*korm;
vehstate_(t,3)=vehsD_(1);
vehstate_(t,1)=vehstate(t-1,1)+v_x*Ts*cos(vehstate_(t,3)); %[1, 2.5, 0, 15];
vehstate_(t,2)=vehstate(t-1,2)+v_x*Ts*sin(vehstate_(t,3));
kimenet=(palya(2,2)-vehstate_(t,2))^2+1.7*(korm-kormanyszog(t-1))^2;
   