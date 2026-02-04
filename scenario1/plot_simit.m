% close all;
Ts = 0.1;
% load results;

new_steps = steps;
new_kormanyszog = kormanyszog(1:(new_steps+1));
new_sebesseg = sebesseg(1:(new_steps+1));

figure; %hold on;
%plot([0:Ts:20*Ts],[0 kormanyszog(2:length(kormanyszog))]*180/pi);
korm=[0:Ts:new_steps*Ts;0 new_kormanyszog(2:length(new_kormanyszog))*180/pi];
j=2;
korm2=[0;0];
for i=0.1:0.01:(new_steps*Ts)
    korm2(1,j)=i;
    korm2(2,j)=0.4*interp1(korm(1,:),korm(2,:),i)+0.6*(korm2(2,j-1));
    j=j+1;
end;
plot(korm2(1,:),korm2(2,:),'LineWidth',2)
xlabel('Time (s)', 'FontSize',12);
ylabel('\delta_{act} (deg)');
ylim([min(korm2(2,:))-0.1, max(korm2(2,:))+0.1]);
xlim([korm2(1,1), korm2(1,end)]);
set(gca,'FontSize',30);


figure; %hold on;
plot([0:Ts:new_steps*Ts],[0 new_sebesseg(2:length(new_sebesseg))]);
seb=[0:Ts:new_steps*Ts;0 new_sebesseg(2:length(new_sebesseg))];
j=2;
seb2=[0;va];
for i=0.1:0.01:(new_steps*Ts)
    seb2(1,j)=i;
    seb2(2,j)=0.4*interp1(seb(1,:),seb(2,:),i)+0.6*(seb2(2,j-1));
    j=j+1;
end;
plot(seb2(1,:),seb2(2,:)*3.6, 'LineWidth',2)
xlabel('Time (s)');
ylabel('v_x (km/h)');
ylim([min(seb2(2,:)*3.6)-0.1, max(seb2(2,:)*3.6)+0.1]);
xlim([seb2(1,1), seb2(1,end)]);
set(gca,'FontSize',30);





