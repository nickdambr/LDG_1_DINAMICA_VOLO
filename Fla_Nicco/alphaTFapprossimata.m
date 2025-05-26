% alpha_TF approssimata, gira prima bode_plots.m



alpha_TF_Num_coeff_Approx=[ -0.278681,-27.3053,0.0000953289]; %from bode.nb
alpha_TF_Den_coeff_Approx=[1,7.35127,36.8055,-0.000128496, ]; %from bode.nb
alpha_TF_Approx = tf ( alpha_TF_Num_coeff_Approx , alpha_TF_Den_coeff_Approx);

optsalpha2 = bodeoptions;
optsalpha2.Title.String = 'Diagrammi di Bode';
optsalpha2.Title.FontSize = 11;
optsalpha2.Title.FontWeight = 'bold';
optsalpha2.Xlabel.String = 'Pulsazione';
optsalpha2.Xlabel.FontSize = 11;
optsalpha2.Ylabel.String = {'Guadagno'  'Fase'};
optsalpha2.Ylabel.FontSize = 11;
optsalpha2.XLim=[10^-2 10^3];
optsalpha2.XLimMode='manual';




figure(13)
bodeplot(alpha_TF_Approx,optsalpha2)
grid