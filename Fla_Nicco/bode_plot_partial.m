% Importazione del sistema dinamico longitudinale creato con il tool di
clear
clc
close all
format long

% Simulink MODEL LINEARIZER

load('linsysLONG_beechcraft99.mat');
[Along, Blong, Clong, Dlong] = ssdata(linsysLONG);   % estrae le matrici

% Bode plot

[ nums , den ] = ss2tf ( Along , Blong , Clong , Dlong,1);
V_TF = tf ( nums (1 ,:) , den );
alpha_TF = tf ( nums (2 ,:) , den );
theta_TF = tf ( nums (4 ,:) , den );

% opzioni grafiche
optsV = bodeoptions;
optsV.Title.String = 'Diagrammi di Bode';
optsV.Title.FontSize = 11;
optsV.Title.FontWeight = 'bold';
optsV.Xlabel.String = 'Pulsazione';
optsV.Xlabel.FontSize = 11;
optsV.Ylabel.String = {'Guadagno'  'Fase'};
optsV.Ylabel.FontSize = 11;
optsV.XLim=[10^-2 100];
optsV.XLimMode='manual';

optsalpha = bodeoptions;
optsalpha.Title.String = 'Diagrammi di Bode';
optsalpha.Title.FontSize = 11;
optsalpha.Title.FontWeight = 'bold';
optsalpha.Xlabel.String = 'Pulsazione';
optsalpha.Xlabel.FontSize = 11;
optsalpha.Ylabel.String = {'Guadagno'  'Fase'};
optsalpha.Ylabel.FontSize = 11;
optsalpha.XLim=[10^-6 100];
optsalpha.XLimMode='manual';

optstheta = bodeoptions;
optstheta.Title.String = 'Diagrammi di Bode';
optstheta.Title.FontSize = 11;
optstheta.Title.FontWeight = 'bold';
optstheta.Xlabel.String = 'Pulsazione';
optstheta.Xlabel.FontSize = 11;
optstheta.Ylabel.String = {'Guadagno'  'Fase'};
optstheta.Ylabel.FontSize = 11;
optstheta.XLim=[10^-16 100];
optstheta.XLimMode='manual';



figure(9)
bodeplot(V_TF,optsV)
grid

figure(10)
bodeplot(alpha_TF,optsalpha)
grid

figure(11)
bodeplot(theta_TF,optstheta)
grid

% funzioni di trasferimento
syms s

V_TF_Num_coeff=V_TF.Numerator{1};
V_TF_Den_coeff=V_TF.Denominator{1};
V_TF_Num = poly2sym(V_TF_Num_coeff, s);
V_TF_Den = poly2sym(V_TF_Den_coeff, s);

alpha_TF_Num_coeff=alpha_TF.Numerator{1};
alpha_TF_Den_coeff=alpha_TF.Denominator{1};
alpha_TF_Num = poly2sym(alpha_TF_Num_coeff, s);
alpha_TF_Den = poly2sym(alpha_TF_Den_coeff, s);

theta_TF_Num_coeff=theta_TF.Numerator{1};
theta_TF_Den_coeff=theta_TF.Denominator{1};
theta_TF_Num = poly2sym(theta_TF_Num_coeff, s);
theta_TF_Den = poly2sym(theta_TF_Den_coeff, s);