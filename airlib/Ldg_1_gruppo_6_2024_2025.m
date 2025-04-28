%% 1° Lavoro di Gruppo

%--------------------------------------------------------------------------

% Corso di Laurea in Ingegneria Aeronautica

% Corso di Dinamica del Volo - Prof. Guido De Matteis

% Studenti: Elisa Jacopucci, Niccolò D'Ambrosio, Francesco Daniele, Matteo
% Grippo

% Gruppo numero: 6

clear
clc
% Scelta del velivolo da analizzare, Nm rappresenta il numero del modello
Ng = 6;
a = 8;
x = floor((Ng-1)/a);
Nm = Ng - x*a;
fprintf(" Il numero del velivolo da analizzare è %d\n",Nm)
disp("Pertanto il modello in questione è il Beechcraft 99")
format long
% Valori di spinta e angolo dell'equlibratore per il Beechcraft 99

Fx0 = 4195.5;    %[N]
de0 = 0.0287194;
dh0 = 0;
W = 3175.2*9.81;     %[N]
S = 26.0129;     % Superficie Alare
b = 14.0208;
cbar = 1.9812;      % Corda media del velivolo
g = 9.81;
m=W/g;
% Per trovare i valori delle variabili di stato e di controllo in
% condizioni di trim si usa la funzione air3m con parametri assegnati per
% il velivolo

V = 95;     % Velocità [m/s]
H = 270;    % Quota [m]
G = 0;      % Angolo di rampa
rho = 1.225*exp(-10^-4*H);

[X0_air3m, U0_air3m] = air3m('airtrim',V,H,G);

% X0_air3m = [95 0.0013 0 0 0 0 0 0.0013 0 0 0 270]
% U0_air3m = [0.380596566005482 0 0 0 0 0 0.0238081811020551 0 0 0]

%Impostazioni delle nuove variabili di spinta e angolo di equilibratore

Fx0_air3m = U0_air3m(1)*10000;
de0_air3m = U0_air3m(7);

% Il vettore X0_air3m è stato usato come condizione inizale per la
% simulazione sul modello Simulink

% Importazione del sistema dinamico longitudinale creato con il tool di
% Simulink MODEL LINEARIZER

Along = [-0.02523       4.024   3.352e-14      -9.806   0.0001158;
         -0.002173      -2.558      0.9608   2.299e-13   9.934e-06;
         1.974e-18      -25.54      -4.792           0  -9.056e-21;
                 0           0           1           0           0;
        -3.25e-16         -95           0          95           0];
 
Blong = [2.381e-13;
           -0.2787;
            -27.03;
                 0;
                 0];
 
Clong = [1   0   0   0   0;
         0   0   0   0   1;
         0   1   0   0   0;
         0   0   1   0   0;
         0   0   0   1   0];
 
Dlong = [0;
         0;
         0;
         0;
         0];
% Si ricorda che il vettore di stato è X=(V zetaE alpha q theta)
% e che il vettore di output (per il sistema longitudinale) è
% Y=(V alpha q theta ZetaE)

% Creazione del sistema linearizzato dalle matrici ottenute

SYSlong = ss(Along,Blong,Clong,Dlong);

% Tracciamento della mappa zero-poli per il sistema longitudinale
figure(1)
pzmap(SYSlong);
%title('Longitudinal Pole Zero Map')
% Calcolo degli autovettori e degli autovalori del sistema linearizzato
format long
[Vlong,EIGlong] = eig(Along);

% Vettore con i moduli degli autovalori

omega_n_long = [abs(EIGlong(1,1));abs(EIGlong(2,2));abs(EIGlong(3,3));...
    abs(EIGlong(4,4));abs(EIGlong(5,5))];

% Pulsazione,periodo e smorzamento associati al modo fugoide

omega_ph = omega_n_long(3);
T_ph = 2*pi/omega_ph;
zita_ph = -real(EIGlong(3,3))/omega_n_long(3);

% Autovettore associato al fugoide

Zph=Vlong(:,3);


% Adimensionalizzazione

Zph(1)=Zph(1)/X0_air3m(1);              % Velocità
Zph(3)=Zph(3)*cbar/(2*X0_air3m(1));     % Velocità di beccheggio
Zph(5)=Zph(5)/X0_air3m(12);             % Quota

% La seconda e la quarta componente di Zlong sono già adimensionali perché
% rappresentano angoli di attacco e beccheggio

% Normalizzazione dei vettori rispetto a theta (angolo di beccheggio)

Zph=Zph./Zph(4);

% Tracciamento del Diagramma di Argand

figure(2)
c0=compass(Zph(1:4));
title('Phugoid Mode Argand Diagram')
legend('V','α','q','θ')
c01=c0(1);
c01.Color='r';
c01.LineWidth=2;
c02=c0(2);
c02.Color='g';
c02.LineWidth=2;
c03=c0(3);
c03.Color='b';
c03.LineWidth=2;
c04=c0(4);
c04.Color='m';
c04.LineWidth=2;

Cd0 = 0.027; Cda = 0.131; Cdq = 0; Cdde = 0; Cdih = 0;
Cl0 = 0.201; Cla = 5.48; Clq = 8.1; Clde = 0.6; Clih = 0;
Cm0 = 0.05; Cma = -1.89; Cmq = -34; Cmde = -2; Cmih = 0;
CY0 = 0; CYb = -0.59; CYp = -0.19; CYr = 0.39; CYda = 0.0; CYdr = 0.148;
Ix = 13673; Iy = 20538; Iz = 31246;

% Confronto con i modelli di ordine ridotto

% Calcolo delle derivate di stabilità

T = Fx0_air3m;
Cde = 2*T/(S*rho*V^2);
Cle = (2*W)/(rho*S*V^2);
Xu = (0.5*rho*V*S*(-3*Cde))/m;
Zu = (-0.5*rho*V*S*(2*Cle))/m;
Mu = 0; %si trascurano gli effetti della comprimibilità
Xw = (0.5*rho*V*S*(Cle-Cda)/m);
Zw = (-0.5*rho*V*S*(Cla+Cde))/m;
Mw = (0.5*rho*V*S*cbar*Cma)/Iy;
Mq = (0.25*rho*S*V*cbar^2*Cmq)/Iy;


% Modello di Lanchester

omega_ph1 = sqrt(2)*(9.81/V);


% Primo modello

Ee = Cle/Cde;
omega_ph2 = omega_ph1;
zita_ph2 = -Xu/(2*omega_ph2);


% Secondo modello

Mwue = Mw*V;
MqZw = Mq*Zw;
omega_ph3_semp = omega_ph2;
zita_ph3_semp = -Xu/(2*omega_ph3s);
omega_ph3 = sqrt(g*(Mu*Zw-Mw*Zu)/(Mw*V-Mq*Zw));
zita_ph3 = -(Xu+Xw*((Mq*Zu-Mu*V)/(Mw*V-Mq*Zw)))/(2*omega_ph3);


% Effetto del gradiente di densità nel fugoide

kappa = 10^(-4);
omega_ph3_prime = omega_ph3 * sqrt(1 + (kappa * W) / (rho * S * g * Cle));


% Modello approssimato corto periodo

omega_SP = sqrt((-0.5*rho*V^2*S*cbar*Cma)/Iy);
zita_SP = -(Mq+Zw)/(2*omega_SP);


% Effetto del gradiente di densità nel corto periodo

omega_SP_prime = omega_SP * sqrt(1 + (kappa * W) / (rho * S * g * Cle));



%------------------------------------------------------------------------

% Dinamica laterodirezionale
% Importazione del sistema dinamico longitudinale creato con il tool di
% Simulink MODEL LINEARIZER

Alat = [-0.274  -0.005251    -0.9866          0     0.1032;
        -18.08     -5.352      1.354          0          0;
         3.774     -0.321    -0.8199          0          0;
             0          0          1          0          0;
             0          1   0.001261          0          0];
 
Blat = [0  0.06874;
   -22.65   0.8048;
   -1.497   -4.798;
        0        0;
        0        0];
 
Clat = [1   0   0   0   0;
        0   1   0   0   0;
        0   0   1   0   0;
        0   0   0   1   0;
        0   0   0   0   1];
 
Dlat = [0   0;
        0   0;
        0   0;
        0   0;
        0   0];

%Gli stati sono (Beta p r psi phi)

% Creazione del sistema linearizzato dalle matrici ottenute

SYSlat=ss(Alat,Blat,Clat,Dlat);

% Tracciamento della mappa zero-poli per il sistema longitudinale
% ----------figure( INSERISCI NUMERO )

pzmap(SYSlat);

%title('Lateral Pole Zero Map')

% Calcolo degli autovettori e degli autovalori del sistema linearizzato

[Vlat,EIGlat]=eig(Alat);

% Vettore con i moduli degli autovalori

omega_n_lat = [abs(EIGlat(1,1));abs(EIGlat(2,2));abs(EIGlat(3,3));...
    abs(EIGlat(4,4));abs(EIGlat(5,5))];

% Modo di Rollio --------------
omega_r = omega_n_lat(2);
T_r = 2*pi/omega_r;
zita_r = -real(EIGlat(2,2))/omega_n_lat(2);

% Autovettore associato al Rollio
Z_r=Vlat(:,2);

% Adimensionalizzazione
Z_r(2)=Z_r(2)*(b/(2*V));
Z_r(3)=Z_r(3)*(b/(2*V));

% Normalizzazione dei vettori rispetto a phi
Z_r=Z_r./Z_r(5);

% Tracciamento del Diagramma di Argand
%%%%%%figure(2)
% c1=compass(Z_r);
% title('Roll Mode Argand Diagram')
% legend('β','p','r','ψ','\phi')
% c11=c1(1);
% c11.Color='r';
% c11.LineWidth=2;
% c12=c1(2);
% c12.Color='g';
% c12.LineWidth=2;
% c13=c1(3);
% c13.Color='b';
% c13.LineWidth=2;
% c14=c1(4);
% c14.Color='m';
% c14.LineWidth=2;
% c15=c1(5);
% c15.Color='y';
% c15.LineWidth=2;

% Confronto con il modello di ordine ridotto
%Lp=


% Modo Spirale ----------------

omega_s = omega_n_lat(3);
T_s = 2*pi/omega_s;
zita_s = -real(EIGlat(3,3))/omega_n_lat(3);

% Autovettore associato alla Spirale
Z_s=Vlat(:,3);
% Adimensionalizzazione
Z_s(2)=Z_s(2)*(b/(2*V));
Z_s(3)=Z_s(3)*(b/(2*V));
% Normalizzazione dei vettori rispetto a phi
Z_r=Z_r./Z_r(5);
% diagramma di Argand non necessario per dinamiche non oscillatorie
% Tracciamento del Diagramma di Argand
%%%%%%figure( )
% c2=compass(Z_s);
% title('Spiral Mode Argand Diagram')
% legend('β','p','r','ψ','\phi')
% c21=c2(1);
% c21.Color='r';
% c21.LineWidth=2;
% c22=c2(2);
% c22.Color='g';
% c22.LineWidth=2;
% c23=c2(3);
% c23.Color='b';
% c23.LineWidth=2;
% c24=c2(4);
% c24.Color='m';
% c24.LineWidth=2;
% c25=c2(5);
% c25.Color='y';
% c25.LineWidth=2;



% Modo di Dutch Roll --------------

omega_dr = omega_n_lat(4);
T_dr = 2*pi/omega_dr;
zita_dr = -real(EIGlat(4,4))/omega_n_lat(4);

% Autovettore associato al Dutch Roll
Z_dr=Vlat(:,4);

% Adimensionalizzazione
Z_dr(2)=Z_dr(2)*(b/(2*V));
Z_dr(3)=Z_dr(3)*(b/(2*V));

% Normalizzazione dei vettori rispetto a phi

Z_dr=Z_dr./Z_dr(5);

% Cerco i moduli per una stima di lunghezza relativa delle frecce nel diagr
% di Argand

Z_dr_mod=abs(Z_dr);

% Tracciamento del Diagramma di Argand
figure(3)
c3=compass(Z_dr);
%title('Dutch Roll Mode Argand Diagram')
legend('β','p','r','ψ','\phi')
c31=c3(1);
c31.Color='r';
c31.LineWidth=2;
c32=c3(2);
c32.Color='g';
c32.LineWidth=2;
c33=c3(3);
c33.Color='b';
c33.LineWidth=2;
c34=c3(4);
c34.Color='m';
c34.LineWidth=2;
c35=c3(5);
c35.Color='y';
c35.LineWidth=2;

% Confronto con il modello di ordine ridotto

N1_r = Alat(3,3);
N1_p = Alat(3,2);
N1_beta = Alat(3,1);
L1_r = Alat(2,3);
L1_p = Alat(2,2);
L1_beta = Alat(2,1);
Yb = Alat(1,1)*V;

% Modo Rollio

lambda_p = L1_p;

% 1° Modello spirale

lambda_s1 = (g/V)*( (N1_r*L1_beta - N1_beta*L1_r)/ (L1_beta));

% 2° Modello spirale

lambda_s2 = (g/V)*( (N1_r*L1_beta - N1_beta*L1_r)/ (N1_beta*L1_p - N1_p*L1_beta));

% 3° Modello spirale

Char_polyn_lat=poly(Alat);
a1_lat=Char_polyn_lat(2);
a2_lat=Char_polyn_lat(3);
a3_lat=Char_polyn_lat(4);
a4_lat=Char_polyn_lat(5);
lambda_s3=-a4_lat/a3_lat;


% Modello approssimato Dutch Roll

omega_DRa = sqrt(N1_beta+N1_r*(Yb/V));
zita_DRa = - ((Yb/V)+N1_r)/(2*omega_DRa);
