% E2. 

% Pick initial values for calculation
% b^2 / S = 9
% S = (1/9) * b^2
% Let b = 50
% Then S = (1/9) * (50)^2 = 277.778

b = 50; %Some value for calculation
b_new = b * 1.05;

S = (1/9) * b^2; %For initial AR of 9
S_new = S * 1.05;

AR = b^2 / S;
AR_new = b_new^2 / S_new;

% C_D = KC_L^2 + C_D0
% C_D - C_D_0 = KC_L^2
% C_Di = KC_L^2
% C_Di = (1/(pi*AR*e)) * C_L^2

% In level flight L = W
% L = W = 1/2 * rho * V^2 * S * C_L
% C_L = W/((1/2) * rho * V^2 * S)
% Since it is given that W does not change and V and rho are constant in
% level flight, we only need to worry about S changing for C_l
% C_L proportional to 1/S

% D = 1/2 * rho * V^2 * S * C_D
% Since only C_D and S change, we don't need to worry about rho and V
% D is proportional to S * C_D

Di_init = getFactor(AR, S);
Di_final = getFactor(AR_new, S_new);

disp("Enrichment Q2: ");
disp("Initial Di: " + Di_init);
disp("New Di: " + Di_final);
percentChange = (Di_final - Di_init)/Di_init * 100;
disp("Percent Change in Di: " + percentChange);

function Di_factor = getFactor(AR, S)
    e = (1.05 + 0.007*pi*AR)^-1;
    C_L_factor = 1/S;
    C_Di_factor = (C_L_factor^2)/(pi*AR*e);
    Di_factor = C_Di_factor * S;
end


