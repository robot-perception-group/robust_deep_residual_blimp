clear 
close all
clc

% Number of measurements: -obs[2], obs[0], -obs[3]
n_e = 3;

% Number of commandos: [yaw, alt, 0, vel]
n_u = 4;

% Vorgaben:
% omega_G = Grenzfrequenz
% S_max   = Obergrenze für |S|
% T_max   = Obergrenze für |T|
% S_0     = Obergrenze S(0)
% T_inf   = Obergrenze für |T| für omega-->unendlich

omega_G  = 0.2;
S_max    = sqrt(2);
S_0      = 0.01;
T_max    = sqrt(2);
T_inf    = 0.01;
omega_zks = 0.5;
KS_max    = 0.5;
KS_min    = 0.01;

% Gewichtungsfunktion W_S (Aufgabe 5-1 a, b)

omega_zs = omega_G;
omega_ns = S_0 * omega_zs / S_max;

A_S = - omega_ns;
B_S = 1;
C_S = (omega_zs - omega_ns) / S_max;
D_S = 1 / S_max;
W_S = ss(A_S, B_S, C_S, D_S);

% Gewichtungsfunktion W_T (Aufgabe 5-1 a, b)

omega_zt = omega_G;
omega_nt = omega_zt * T_max / T_inf;

A_T = - omega_nt;
B_T = 1;
C_T = (omega_zt - omega_nt) / T_inf;
D_T = 1 / T_inf;
W_T = ss(A_T, B_T, C_T, D_T);

% Gewichtungsfunktion W_KS

omega_nks = omega_zks * KS_max / KS_min;

A_KS = - omega_nks;
B_KS = 1;
C_KS = (omega_zks - omega_nks) / KS_min;
D_KS = 1 / KS_min;
W_KS = ss(A_KS, B_KS, C_KS, D_KS);

% WS(W1)...E->Z1; WKS(W2)...U->Z2(x2); WT(W3)...Y->Z3
% num_s = [1/S_max, omega_zs/S_max];
% den_s = [1, omega_ns];
% num_t = [1/T_min, omega_zt/T_min];
% den_t = [1, omega_nt];
% 
% W_S = tf(num_s, den_s);
% W_T = tf(num_t, den_t);

% A = zeros(4,4);
% B = [-5, -2.75, -5.27, 5;
%     0, -0.11, -0.46, 0;
%     0, -0.28, -0.77, 0;
%     -25, 0, 0, 0];
% C = eye(4,4);
% D = zeros(4,4);

A = zeros(3,3);
B = [0, 0, -5, 9.8;
     0,-0.4, -0.77, 0;
    -15, 0, 0, 0];
% B = [-5.6, -2.75, -5.27, 9.8;
%     1, -0.28, -0.77, -10.26;
%     -25, 0, 0, 0];
% B = [0, 0, 5;
%     0, -0.2, -3;
%     -18, 0, 0];
C = eye(3,3);
C(1,1) = -1;
D = zeros(3,4);

% G...plant
G_nominal = ss(A,B,C,D);
[num,den] = pade(0.55,1);
G_delay = tf(num,den);
G = series(G_nominal, G_delay);
% P...augmented plant
P = augw(G, W_S, W_KS, W_T);

% K... controller, CL... closed-loop system, gamma... infinity norm of closed loop system
opts = hinfsynOptions('Display','on');
[K, CL, gamma, info] = hinfsyn(P, n_e, n_u, opts);

A = K.A;
B = K.B;
C = K.C;
D = K.D;

transfer = tf(K);
% Test if controller will return silent (examine if stationary state will be achieved) 
% T = feedback(K,G);
% step(T)
% impulse(K)
% G_0: offener Kreis G*K
% T: geschlossener Kreis, Führungsübertragungsfunktion
% S: geschlossener Kreis, Sensitivitätsfunktion
% KS: K*S

G_0 = series(K, G);
R = eye(3);
T   = feedback(G_0, R);
S   = 1 - T;
KS  = series(S, K);

% Untersuchung der Lösung:
% zero_K, poles_K: Nullstellen und Pole von K
% zero_T, poles_T: Nullstellen und Pole von T
% T0: T(0), stationärer Übertragungsfaktor von T
% fb: Bandbreite des geschlossenen Kreises
% Gm, Pm: Amplituden- und Phasenreserve
% Wcp: Durchtrittsfrequenz von G_0

zero_K = zero(K);
[omega_K, zeta_K, poles_K] = damp(K);
zero_T = zero(T);
[omega_T, zeta_T, poles_T] = damp(T);
T0 = evalfr(T, 0);
% fb = bandwidth(T);
% [Gm, Pm, Wcg, Wcp] = margin(G_0);

