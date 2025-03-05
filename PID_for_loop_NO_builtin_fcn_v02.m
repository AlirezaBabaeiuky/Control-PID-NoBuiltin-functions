% 
% Building PID controller using for loop (iterative loop) and NO Built-In fcn.  
clc; clear all; close all; 
m = 1; 
k= 1e4;
ccr = 2 * sqrt(m*k)
zeta = 0.01;
c = zeta*ccr

kp = 1000;
ki = 50;
kd = 10;

ref = ones(1, 10000); 
pos = 0;
vel = 0; 
preve = 0;
ing = 0; 
dt = 0.001;

for i = 1 : 10000 % this loop is to complete the time trace in control signal 
    e = ref(i) - pos;
    ing = ing + e*dt;
    der = (e - preve) / dt;
    preve = e; 
    ctrlsig = kp*e + ki*ing + kd*der;

    acc = (1/m) * (ctrlsig - c*vel - k*pos);
    vel = acc*dt + vel; 
    pos = pos + vel*dt; 

    C(i) = ctrlsig; 
    Acc(i) = acc;
    Vel(i) = vel;
    Pos(i) = pos; 
    E(i) = e; 

    fprintf("error = %f; control signal = %f; Position/Response = %f\n", e, ctrlsig, pos) % % is format specifier, /n is escape sequence 

end
fprintf("/n")
% fprintf("error = %f /n; control signal = %f /n; Position/Response = %f \n", E, C, Pos) % % is format specifier, /n is escape sequence 
ess = ref(end) - Pos(end) 

figure(1)
sgtitle('PID no built-in', 'FontSize', 22)
subplot(3, 1, 1)
plot((1:10000)*dt, Pos)
title('Position')
grid on 
subplot(3, 1, 2)
plot((1:10000)*dt, C)
title('Controller Effort')
grid on 
subplot(3, 1, 3)
plot((1:10000)*dt, E)
title('Error')
grid on 









