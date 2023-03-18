clear 
close all
clc

% Number of measurements: delta_theta
n_e = 1;

% Number of commandos: angular.z
n_u = 1;

% Vorgaben:
% omega_G = Grenzfrequenz
% S_max   = Obergrenze für |S|
% T_max   = Obergrenze für |T|
% S_0     = Obergrenze S(0)
% T_inf   = Obergrenze für |T| für omegda-->unendlich

% omega_G  = 20;
% S_max    = sqrt(10);
% S_0      = 0.001;
% T_max    = sqrt(10);
% T_inf    = 0.01;
% omega_zks = 20;
% KS_max    = 10;
% KS_min    = 0.01;

omega_G  = 25;
S_max    = sqrt(8);
S_0      = 0.001;
T_max    = sqrt(5);
T_inf    = 0.01;
omega_zks = 200;
KS_max    = 10;
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

% x = R; y = R; u = v; w = 0
A1 = 0;
B1 = -1;
C1 = 1;
D1 = 0;
% x = theta; y = theta; u = omega; w = sigma
A2 = 0;
B2 = 1;
C2 = 1;
D2 = 0;

% MIMO Turtle
A3 = zeros(2,2);
B3 = [-1 0; 0 1];
C3 = eye(2);
D3 = zeros(2,2);
 
% G...plant (G1... v plant; G2... omega plant)
G1 = ss(A1, B1, C1, D1);
G2 = ss(A2, B2, C2, D2);
G3 = ss(A3, B3, C3, D3);

% P...augmented plant
P1 = augw(G1, W_S, W_KS, W_T);
P2 = augw(G2, W_S, W_KS, W_T);
P3 = augw(G3, W_S, W_KS, W_T);

% K... controller, CL... closed-loop system, gamma... infinity norm of closed loop system
opts = hinfsynOptions('Display','on');
[K1, CL1, gamma1, info1] = hinfsyn(P1, n_e, n_u, opts);
[K2, CL2, gamma2, info2] = hinfsyn(P2, n_e, n_u, opts);
[K3, CL3, gamma3, info3] = hinfsyn(P3, 2, 2, opts);

% Test if controller will return silent (examine if stationary state will be achieved) 
% T = feedback(K,G);
% step(T)
% impulse(K)
% G_0: offener Kreis G*K
% T: geschlossener Kreis, Führungsübertragungsfunktion
% S: geschlossener Kreis, Sensitivitätsfunktion
% KS: K*S

% G_0 = series(K2, G2);
% T   = feedback(G_0, 1);
% S   = 1 - T;
% KS  = series(S, K2);
% 
% % Untersuchung der Lösung:
% % zero_K, poles_K: Nullstellen und Pole von K
% % zero_T, poles_T: Nullstellen und Pole von T
% % T0: T(0), stationärer Übertragungsfaktor von T
% % fb: Bandbreite des geschlossenen Kreises
% % Gm, Pm: Amplituden- und Phasenreserve
% % Wcp: Durchtrittsfrequenz von G_0
% 
% zero_K = zero(K2);
% [omega_K, zeta_K, poles_K] = damp(K2);
% zero_T = zero(T);
% [omega_T, zeta_T, poles_T] = damp(T);
% T0 = evalfr(T, 0);
% fb = bandwidth(T);
% [Gm, Pm, Wcg, Wcp] = margin(G_0);

K1_A = K1.A;
K1_B = K1.B;
K1_C = K1.C;
K1_D = K1.D;

K2_A = K2.A;
K2_B = K2.B;
K2_C = K2.C;
K2_D = K2.D;

K3_A = K3.A;
K3_B = K3.B;
K3_C = K3.C;
K3_D = K3.D;

% options = bodeoptions;
% figure
% bodemag(1 / W_T, 1 / W_KS, 1 / W_S)
% legend('$|\frac{1}{W\_{T}}|$','$|\frac{1}{W\_{KS}}|$','$|\frac{1}{W\_{S}}|$','fontsize',24,'interpreter','latex')

% T1 = feedback(series(K1, G1),eye(n_e));
% S1 = 1 - T1;
% SG1 = series(G1, S1);
% figure
% bodemag(W_T)
% bodemag(SG1)
% frspSG1 = evalfr(SG1,10)


% T2 = feedback(series(K2, G2),eye(1));
% S2 = 1 - T2;
% SG2 = series(G2, S2);
% figure
% bodemag(SG2)
% frspSG2 = evalfr(SG2,10)


w_t = norm(evalfr(W_T, j*100))
db = mag2db(w_t)

