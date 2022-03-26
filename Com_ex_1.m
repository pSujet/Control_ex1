%% Computer exercise 1
% Sujet Phodapol 2022

%% clear
clc;
clear all;
close all;
%% ===== 4.1 =====
%% Plant
s = tf('s');
G = 3*(-s+1)/((5*s+1)*(10*s+1));
figure, bode(G);

%% design controller
figure, bode(G)
hold on
% Lead controller
wc = 0.4;
[m,phase] = bode(G,wc);
phi = 30 - (phase - 180); % -> to graph beta
beta = 0.5; % beta = 1/a
Td = 1/(wc*sqrt(beta));
F_lead = (Td *s +1)/(beta*Td*s+1);
[m,phase] = bode(F_lead*G,wc);
bode(F_lead*G)
K = 1/m;
bode(K*F_lead*G)

% Lag controller
gamma = 0.1; %beta; %small -> gain bigger at low frequency -> smaller 
TI = 10/wc;
F_lag = (TI*s+1)/(TI*s+gamma);
bode(K*F_lead*F_lag*G);
figure,margin(K*F_lead*F_lag*G);
sysc = K*F_lead*F_lag;
syscl = feedback(sysc*G,1);
figure,step(syscl);

%% Bandwidth
bandwidth(syscl);
figure,step(syscl);
stepinfo(syscl)
peak = getPeakGain(syscl); % magnitude
Mt = 20*log10(peak); % in dB

%% ===== 4.2 =====


















%% design controller
phi = 12;
w = 0.4;
a = (1+sind(phi))/(1-sind(phi));
T = 1/(w*sqrt(a));
sysle = 1/a*(a*T*s+1)/(T*s+1);
hold on;
sysg = G*sysle;
bode(sysg);

%% Lag
T2 = 10/w;
b = a;
syslg = b*(T2*s+1)/(b*T2*s+1);
sysg2 = sysg*syslg;
bode(sysg2);
[m,phase] = bode(sysg2,w);
%phase margin > 30, cross over 0.4 rad
%% step
L = 1/m;
bode(L*sysg2);
syscl = feedback(L*sysg2,1);
figure,margin(L*sysg2);
figure, step(syscl)



