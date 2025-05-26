% Importazione del sistema dinamico longitudinale creato con il tool di
clear
clc
close all
format long

% Simulink MODEL LINEARIZER

load('linsysLONG_beechcraft99.mat');
[Along, Blong, Clong, Dlong] = ssdata(linsysLONG);   % estrae le matrici

% Bode plots
% X=[V, H, alpha, q, theta]
[ nums , den ] = ss2tf ( Along , Blong , Clong , Dlong,1);
V_TF = tf ( nums (1 ,:) , den );
alpha_TF = tf ( nums (3 ,:) , den );
theta_TF = tf ( nums (5 ,:) , den );

% modello ottenuto trascurando l'effetto del gradiente di densità
Along_prime = Along;
Blong_prime = Blong;
Clong_prime = Clong;
Dlong_prime = Dlong;

Along_prime(5,:)=[];
Along_prime(:,5)=[];
Blong_prime(5)=[];
Clong_prime(2,:)=[];
Clong_prime(:,2)=[];
Dlong_prime(2)=[];

[ nums_prime , den_prime ] = ss2tf ( Along_prime , Blong_prime , Clong_prime , Dlong_prime,1);
V_TF_prime = tf ( nums_prime (1 ,:) , den_prime );


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
optsalpha.XLim=[10^-2 1000];
optsalpha.XLimMode='manual';

optstheta = bodeoptions;
optstheta.Title.String = 'Diagrammi di Bode';
optstheta.Title.FontSize = 11;
optstheta.Title.FontWeight = 'bold';
optstheta.Xlabel.String = 'Pulsazione';
optstheta.Xlabel.FontSize = 11;
optstheta.Ylabel.String = {'Guadagno'  'Fase'};
optstheta.Ylabel.FontSize = 11;
optstheta.XLim=[10^-2 10];
optstheta.XLimMode='manual';

optsV_prime = bodeoptions;
optsV_prime.Title.String = 'Diagrammi di Bode';
optsV_prime.Title.FontSize = 11;
optsV_prime.Title.FontWeight = 'bold';
optsV_prime.Xlabel.String = 'Pulsazione';
optsV_prime.Xlabel.FontSize = 11;
optsV_prime.Ylabel.String = {'Guadagno'  'Fase'};
optsV_prime.Ylabel.FontSize = 11;
optsV_prime.XLim=[10^-8 100];
optsV_prime.XLimMode='manual';



figure(9)
bodeplot(V_TF,optsV)
grid

figure(10)
bodeplot(alpha_TF,optsalpha)
grid

figure(11)
bodeplot(theta_TF,optstheta)
grid

figure(12)
bodeplot(V_TF,optsV_prime)
hold on
bodeplot(V_TF_prime,optsV_prime)
grid
legend('Modello iniziale', 'Modello ottenuto trascurando l''effetto del gradiente di densità')

% funzioni di trasferimento
syms s

TF_Den_coeff = V_TF.Denominator{1}; % usiamo V_TF, ma il denominatore è uguale 
r_TF_Den = roots(TF_Den_coeff);
TF_Den_factored = vpa(expand_roots_into_factors(r_TF_Den),4);
disp(TF_Den_factored)

V_TF_Num_coeff = V_TF.Numerator{1};
r_V_TF_Num = roots(V_TF_Num_coeff);
kV = V_TF_Num_coeff(find(V_TF_Num_coeff ~= 0, 1)); % fattore moltiplicativo del numeratore
V_TF_Num_factored = vpa(kV*expand_roots_into_factors(r_V_TF_Num),4);
disp(V_TF_Num_factored)


alpha_TF_Num_coeff=alpha_TF.Numerator{1};
r_alpha_TF_Num = roots(alpha_TF_Num_coeff);
kalpha = alpha_TF_Num_coeff(find(alpha_TF_Num_coeff ~= 0, 1)); % fattore moltiplicativo del numeratore
alpha_TF_Num_factored = vpa(kalpha*expand_roots_into_factors(r_alpha_TF_Num),4);
disp(alpha_TF_Num_factored)



theta_TF_Num_coeff=theta_TF.Numerator{1};
r_theta_TF_Num = roots(theta_TF_Num_coeff);
ktheta = theta_TF_Num_coeff(find(theta_TF_Num_coeff ~= 0, 1)); % fattore moltiplicativo del numeratore
theta_TF_Num_factored = vpa(ktheta*expand_roots_into_factors(r_theta_TF_Num),4);
disp(theta_TF_Num_factored)





