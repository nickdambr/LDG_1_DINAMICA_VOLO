% Importazione del sistema dinamico longitudinale creato con il tool di
clear
clc
close all

% Simulink MODEL LINEARIZER

load('linsysLONG_beechcraft99.mat');
[Along, Blong, Clong, Dlong] = ssdata(linsysLONG);   % estrae le matrici

% Bode plot

[ nums , den ] = ss2tf ( Along , Blong , Clong , Dlong,1);
V_TF = tf ( nums (1 ,:) , den );
alpha_TF = tf ( nums (2 ,:) , den );
theta_TF = tf ( nums (4 ,:) , den );

% opzioni grafiche
opts = bodeoptions;

opts.Title.String = 'System Frequency Response';
opts.FreqUnits = 'Hz';

figure(9)
bodeplot(V_TF)
grid

figure(10)
bodeplot(alpha_TF)
grid

figure(11)
bodeplot(theta_TF)
grid