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