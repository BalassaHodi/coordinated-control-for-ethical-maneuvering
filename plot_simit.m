close all;
Ts=0.1;
load results;

figure; %hold on;
%plot([0:Ts:20*Ts],[0 kormanyszog(2:length(kormanyszog))]*180/pi);
korm=[0:Ts:15*Ts;0 kormanyszog(2:length(kormanyszog))*180/pi];
j=2;
korm2=[0;0];
for i=0.1:0.01:2
    korm2(1,j)=i;
    korm2(2,j)=0.4*interp1(korm(1,:),korm(2,:),i)+0.6*(korm2(2,j-1));
    j=j+1;
end;
plot(korm2(1,:),korm2(2,:))

figure; %hold on;
plot([0:Ts:15*Ts],[0 sebesseg(2:length(sebesseg))]);
seb=[0:Ts:15*Ts;0 sebesseg(2:length(sebesseg))];
j=2;
seb2=[0;60/3.6];
for i=0.1:0.01:2
    seb2(1,j)=i;
    seb2(2,j)=0.4*interp1(seb(1,:),seb(2,:),i)+0.6*(seb2(2,j-1));
    j=j+1;
end;
plot(seb2(1,:),seb2(2,:)*3.6)




