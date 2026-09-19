% E1. 
AR = 9;
AR_new = 9 * 1.05;

% C_D = KC_L^2 + C_D0
% C_D - C_D_0 = KC_L^2
% C_Di = KC_L^2
% C_Di = (1/(pi*AR*e)) * C_L^2

% In level flight L = W
% L = W = 1/2 * rho * V^2 * S * C_L
% C_L = W/((1/2) * rho * V^2 * S)
% Since it is given that W and S does not change and level flight will have
% consistent V and rho, we can keep C_L as a constant 

C_di_init = getFactor(AR);
C_di_final = getFactor(AR_new);

disp("Enrichment Q1: ");
disp("Initial C_Di: " + C_di_init);
disp("New C_Di: " + C_di_final);
percentChange = (C_di_final - C_di_init)/C_di_init * 100;
disp("Percent Change in C_Di: " + percentChange);

function C_Di_factor = getFactor(AR)
    e = (1.05 + 0.007*pi*AR)^-1;
    C_Di_factor = 1/(pi*AR*e);
end


