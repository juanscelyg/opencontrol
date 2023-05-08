figure
t=A(:,1)/1000;
vego=A(:,2);
%hud=A(:,3);
vpid=A(:,4);
%plot(t,vego,t,hud,t,vpid)
plot(t,vego,t,vpid)
hold on
grid on
ylabel('Velocity (m/s)')
xlabel('time (s)')
%legend('v\_ego','hud','v\_pid')
legend('v\_ego','v\_pid')