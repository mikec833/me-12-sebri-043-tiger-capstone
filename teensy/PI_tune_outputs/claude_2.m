clear; close all; clc;
T   = readtable("motor_id_l_20260917_171906.csv");
Ts  = 0.01;          % sample time (s)
CPR = 64*70;         % counts per wheel rev

t = T.t_ms/1000;
u = T.cmdL;                                   % command (%)
w = [0; diff(T.countL)] * 2*pi/(CPR*Ts);      % wheel speed (rad/s)

figure
yyaxis left;  plot(t, w);   ylabel('\theta dot (rad/s)')
yyaxis right; stairs(t, u); ylabel('command (%)')
xlabel('t (s)')