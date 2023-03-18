clear 
close all
clc

% Number of measurements: -obs[2], obs[0], -obs[3]
n_e = 2;

% Number of commandos: [yaw, alt, 0, vel]
n_u = 3;

% Vorgaben:
% omega_G = Grenzfrequenz
% S_max   = Obergrenze für |S|
% T_max   = Obergrenze für |T|
% S_0     = Obergrenze S(0)
% T_inf   = Obergrenze für |T| für omega-->unendlich

omega_G1  = 0.2;
S_max1    = sqrt(2);
S_01      = 0.01;
T_max1    = sqrt(2);
T_inf1    = 0.01;
omega_zks1 = 0.5;
KS_max1    = 0.5;
KS_min1    = 0.01;

% Gewichtungsfunktion W_S (Aufgabe 5-1 a, b)

omega_zs1 = omega_G1;
omega_ns1 = S_01 * omega_zs1 / S_max1;

A_S1 = - omega_ns1;
B_S1 = 1;
C_S1 = (omega_zs1 - omega_ns1) / S_max1;
D_S1 = 1 / S_max1;
W_S1 = ss(A_S1, B_S1, C_S1, D_S1);

% Gewichtungsfunktion W_T (Aufgabe 5-1 a, b)

omega_zt1 = omega_G1;
omega_nt1 = omega_zt1 * T_max1 / T_inf1;

A_T1 = - omega_nt1;
B_T1 = 1;
C_T1 = (omega_zt1 - omega_nt1) / T_inf1;
D_T1 = 1 / T_inf1;
W_T1 = ss(A_T1, B_T1, C_T1, D_T1);

% Gewichtungsfunktion W_KS

omega_nks1 = omega_zks1 * KS_max1 / KS_min1;

A_KS1 = - omega_nks1;
B_KS1 = 1;
C_KS1 = (omega_zks1 - omega_nks1) / KS_min1;
D_KS1 = 1 / KS_min1;
W_KS1 = ss(A_KS1, B_KS1, C_KS1, D_KS1);

% WS(W1)...E->Z1; WKS(W2)...U->Z2(x2); WT(W3)...Y->Z3
% num_s = [1/S_max, omega_zs/S_max];
% den_s = [1, omega_ns];
% num_t = [1/T_min, omega_zt/T_min];
% den_t = [1, omega_nt];
% 
% W_S = tf(num_s, den_s);
% W_T = tf(num_t, den_t);


A1 = zeros(2,2);
% B1 = [-3, 10;
%      -0.28, -10];
B1 = [0, -5, 9.8;
    -0.4, -0.77, 0];
% B = [0, 0, 5;
%     0, -0.2, -3;
%     -18, 0, 0];
C1 = eye(2,2);
C1(1,1) = -1;
D1 = zeros(2,3);
% A1 = zeros(4,4);
% A1(1,3) = 1;
% A1(2,4) = 1;
% B1 = zeros(4,3);
% B1(3,2) = -0.35;
% B1(3,3) = 0.9;
% B1(4,1) = -0.1;
% B1(4,2) = -0.3;
% C1 = eye(4);
% C1(1,1) = -1;
% C1(3:4,:) = [];
% D1 = zeros(2,3);



%  Vorgaben:
% omega_G = Grenzfrequenz
% S_max   = Obergrenze für |S|
% T_max   = Obergrenze für |T|
% S_0     = Obergrenze S(0)
% T_inf   = Obergrenze für |T| für omega-->unendlich

omega_G2  = 0.2;
S_max2    = sqrt(2);
S_02      = 0.01;
T_max2    = sqrt(2.2);
T_inf2    = 0.01;
omega_zks2 = 0.8;
KS_max2    = 0.8;
KS_min2    = 0.01;

% Gewichtungsfunktion W_S (Aufgabe 5-1 a, b)

omega_zs2 = omega_G2;
omega_ns2 = S_02 * omega_zs2 / S_max2;

A_S2 = - omega_ns2;
B_S2 = 1;
C_S2 = (omega_zs2 - omega_ns2) / S_max2;
D_S2 = 1 / S_max2;
W_S2 = ss(A_S2, B_S2, C_S2, D_S2);

% Gewichtungsfunktion W_T (Aufgabe 5-1 a, b)

omega_zt2 = omega_G2;
omega_nt2 = omega_zt2 * T_max2 / T_inf2;

A_T2 = - omega_nt2;
B_T2 = 1;
C_T2 = (omega_zt2 - omega_nt2) / T_inf2;
D_T2 = 1 / T_inf2;
W_T2 = ss(A_T2, B_T2, C_T2, D_T2);

% Gewichtungsfunktion W_KS

omega_nks2 = omega_zks2 * KS_max2 / KS_min2;

A_KS2 = - omega_nks2;
B_KS2 = 1;
C_KS2 = (omega_zks2 - omega_nks2) / KS_min2;
D_KS2 = 1 / KS_min2;
W_KS2 = ss(A_KS2, B_KS2, C_KS2, D_KS2);

A2 = 0;
B2 = -20;
C2 = 1;
D2 = 0;

% A2 = [0 1;
%       0 0];
% B2 = [0;
%       -10];
% C2 = [1 0];
% D2 = 0;

% A2 = [0 1;
%       0 0];
% B2 = [0;
%       -20];
% C2 = [1 0];
% D2 = 0;

W_S3 = ss(A_S1, B_S1, C_S1, D_S1);

% Gewichtungsfunktion W_T (Aufgabe 5-1 a, b)

W_T3 = ss(A_T1, B_T1, C_T1, D_T1);

% Gewichtungsfunktion W_KS

W_KS3 = ss(A_KS1, B_KS1, C_KS1, D_KS1);

% WS(W1)...E->Z1; WKS(W2)...U->Z2(x2); WT(W3)...Y->Z3
% num_s = [1/S_max, omega_zs/S_max];
% den_s = [1, omega_ns];
% num_t = [1/T_min, omega_zt/T_min];
% den_t = [1, omega_nt];
% 
% W_S = tf(num_s, den_s);
% W_T = tf(num_t, den_t);


A3 = zeros(2,2);
% B1 = [-3, 10;
%      -0.28, -10];
B3 = [0, 5, -9.8;
    -0.4, 0.77, 0];
% B = [0, 0, 5;
%     0, -0.2, -3;
%     -18, 0, 0];
C3 = eye(2,2);
C3(1,1) = -1;
D3 = zeros(2,3);

% G...plant
G1 = ss(A1,B1,C1,D1);
G2_nominal = ss(A2,B2,C2,D2);
[num,den] = pade(0.65,2); 
G2_delay = tf(num,den);
G2 = series(G2_nominal, G2_delay);
G3 = ss(A3,B3,C3,D3);
% P...augmented plant
P1 = augw(G1, W_S1, W_KS1, W_T1);
P2 = augw(G2, W_S2, W_KS2, W_T2);
P3 = augw(G3, W_S3, W_KS3, W_T3);

% K... controller, CL... closed-loop system, gamma... infinity norm of closed loop system
opts = hinfsynOptions('Display','on');
[K1, CL1, gamma1, info1] = hinfsyn(P1, n_e, n_u, opts);
[K2, CL2, gamma2, info2] = hinfsyn(P2, 1, 1, opts);
[K3, CL3, gamma3, info3] = hinfsyn(P1, n_e, n_u, opts);

Kas_A = K1.A;
Kas_B = K1.B;
Kas_C = K1.C;
Kas_D = K1.D;

Kya_A = K2.A;
Kya_B = K2.B;
Kya_C = K2.C;
Kya_D = K2.D;

Kds_A = K3.A;
Kds_B = K3.B;
Kds_C = K3.C;
Kds_D = K3.D;

% transfer = tf(K);
% % Test if controller will return silent (examine if stationary state will be achieved) 
% % T = feedback(K,G);
% % step(T)
% % impulse(K)
% % G_0: offener Kreis G*K
% % T: geschlossener Kreis, Führungsübertragungsfunktion
% % S: geschlossener Kreis, Sensitivitätsfunktion
% % KS: K*S
% 
% G_0 = series(K, G);
% R = eye(3);
% T   = feedback(G_0, R);
% S   = 1 - T;
% KS  = series(S, K);
% 
% % Untersuchung der Lösung:
% % zero_K, poles_K: Nullstellen und Pole von K
% % zero_T, poles_T: Nullstellen und Pole von T
% % T0: T(0), stationärer Übertragungsfaktor von T
% % fb: Bandbreite des geschlossenen Kreises
% % Gm, Pm: Amplituden- und Phasenreserve
% % Wcp: Durchtrittsfrequenz von G_0
% 
% zero_K = zero(K);
% [omega_K, zeta_K, poles_K] = damp(K);
% zero_T = zero(T);
% [omega_T, zeta_T, poles_T] = damp(T);
% T0 = evalfr(T, 0);
% % fb = bandwidth(T);
% % [Gm, Pm, Wcg, Wcp] = margin(G_0);
% 

% T1 = feedback(series(K1, G1),eye(n_e));
% S1 = 1 - T1;
% SG1 = series(G1, S1);
% figure
% bodemag(SG1)
% frspSG1 = evalfr(SG1,10)
% 
% 
% T2 = feedback(series(K2, G2),eye(1));
% S2 = 1 - T2;
% SG2 = series(G2, S2);
% figure
% bodemag(SG2)
% frspSG2 = evalfr(SG2,10)
% 
% T3 = feedback(series(K3, G3),eye(n_e));
% S3 = 1 - T3;
% SG3 = series(G3, S3);
% figure
% bodemag(SG3)
% frspSG3 = evalfr(SG3,10)

% figure
% bodemag(W_T1)


w_t1 = norm(evalfr(W_T1, j*10))
db = mag2db(w_t1)
