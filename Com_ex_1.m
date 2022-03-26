%% Control 2022
% Computer exercise 1
% Sujet Phodapol 
% Matthew Lock

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
%% plant 
s = tf('s');
G = 20/((s+1)*((s/20)^2+s/20+1));

%% requirement
%rise time less than 0.2 overshoot less than 10%
wn = 1.8/0.2; %9
M = 0.1;
xi = sqrt(log(M)^2/(pi^2+log(M)^2));
phi = 100*xi; %59.1155

%% 4.2.1 Design controller
wc = 1;
Fy = 1/G*wc/s;
% add two poles at very far from img axis
P = 1/(s/100+1)^2;
% compute L
L = 10*Fy*G*P; % 10 = gain
syscl = feedback(L,1);
step(syscl)

%% 4.2.2 Disturbance Attenuation
% scaled disturbance
Gd = 10/(s+1);
% design controller
wI = 10;
Fy = (s+wI)/s/G*Gd * 1/(s/100+1)^2;
% compute L
L = Fy*G;
% sensitivity
S = 1/(1+L);
SGd = S*Gd;
% step response of disturbance
figure,step(SGd);
% step response of close loop system
syscl = feedback(L,1);
figure,step(syscl);

%% 4.2.3 Reference tracking specifications
%%
bode(L)
hold on

%%
LL = 10*L;
syscl = feedback(LL,1);
figure,step(minreal(syscl));
% sensitivity
S = 1/(1+LL);
SGd = S*Gd;
% step response of disturbance
figure,step(SGd);

%%
Fr = 1;
r = 0;
d = 0;
u = Fy*Fr*S*r - Fy*Gd*S*d;
figure,step(Fy*Gd*S)

%% Lead controller
wc = 10;
[m,phase] = bode(G,wc);
phi = 30;
a = (1+sind(phi))/(1-sind(phi));
beta = 1/a;
Td = 1/(wc*sqrt(beta));
F_lead = (Td *s +1)/(beta*Td*s+1);
[m,phase] = bode(F_lead*L,wc);
bode(F_lead*L)

%%
LLL = F_lead*L;
syscl = feedback(LLL,1);
figure,step(syscl);
% sensitivity
S = 1/(1+LLL);
SGd = S*Gd;
% step response of disturbance
figure,step(SGd);



%%
hold on
K = 1/m;
bode(K*F_lead*L)

%%
K = 1;
L = K*F_lead*L;
syscl = feedback(L,1);
figure,step(syscl);

% sensitivity
S = 1/(1+L);
SGd = S*Gd;
% step response of disturbance
figure,step(SGd);

%% Prefiltering
tau = 1;
Fr = 1/(1+tau*s);



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



