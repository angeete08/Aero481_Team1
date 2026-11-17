%% V-n Diagram (Based on Martins' Metabook, Sec. 11.3.4)
clear; clc; close all;

% --- 1. Aircraft & Atmospheric Parameters ---
rho_SL = 1.225;          % Sea-level air density [kg/m^3]
rho    = 1.189;          % Flight altitude air density [kg/m^3]
sigma  = rho / rho_SL;   % Density ratio (Eq. 11.1)
a_snd  = 295.1;          % Speed of sound at flight altitude [m/s] at 12000ft
g      = 9.81;           % Gravity [m/s^2]

W_lbs  = 16682;          % Aircraft weight [lbs]
m_kg   = 7566.9;         % Aircraft mass [kg]
W_N    = m_kg * g;       % Aircraft weight [N]

S      = 92.94;          % Wing reference area [m^2]
S_ft2  = 1000.3978;      % Wing reference area [ft^2]
b      = 27.27;          % Wing span [m]
AR     = b^2 / S;        % Wing aspect ratio (~8.00)
c_bar  = S / b;          % Mean geometric chord [m]
WS_SI  = W_N / S;        % Wing loading [N/m^2]
WS_imp = W_lbs / S_ft2;  % Wing loading [lbs/ft^2]

% --- 2. Aerodynamic & Design Speed Inputs ---
Cl_max    = 1.1;         % Clean max lift coefficient
Cl_min    = -1.4;        % Negative max lift coefficient (calculated from Xfoil)
Lambda_c2 = 0;           % Mid-chord sweep angle [deg] (update if swept)
kappa     = 0.97;        % Empirical section lift-curve factor (Eq. 11.6)

% Unit conversion factors & Eq. (11.1) airspeed conversions
ms_to_kts  = 1.94384;
kts_to_ms  = 1 / ms_to_kts;
eas_to_tas = @(V_eas) V_eas ./ sqrt(sigma); % Eq. (11.1): V_TAS = V_EAS / sqrt(sigma)
tas_to_eas = @(V_tas) V_tas .* sqrt(sigma); % Eq. (11.1): V_EAS = V_TAS * sqrt(sigma)

% Design Airspeeds in EAS [knots] -> UPDATE VC_eas_kts (or convert from VC_tas_kts)
VC_eas_kts = 225;                % Design cruise speed V_C [knots EAS] (placeholder)
VD_eas_kts = 1.25 * VC_eas_kts;  % Design dive speed V_D [knots EAS]

% Aircraft under 50,0000 lbs 
n_limit_pos = min(3.8, max(2.5, 2.1 + 24000 / (W_lbs + 10000)));
n_limit_neg = -1.0;      % FAR Part 25 negative limit load factor

fprintf('Aspect Ratio (AR): %.2f\n', AR);
fprintf('Positive Limit Load Factor (n_limit): %.4f g\n', n_limit_pos);

% --- 4. Key Maneuver Airspeeds (Eq. 11.2, in EAS) ---
VS_pos_eas_ms  = sqrt((2 * WS_SI) / (rho_SL * Cl_max));
VS_neg_eas_ms  = sqrt((2 * WS_SI) / (rho_SL * abs(Cl_min)));
VA_eas_ms      = VS_pos_eas_ms * sqrt(n_limit_pos);
VA_neg_eas_ms  = VS_neg_eas_ms * sqrt(abs(n_limit_neg));

VS_pos_eas_kts = VS_pos_eas_ms * ms_to_kts;
VA_eas_kts     = VA_eas_ms * ms_to_kts;
VA_neg_eas_kts = VA_neg_eas_ms * ms_to_kts;

% --- 5. DATCOM Lift-Curve Slope & Gust Helper Functions (Eqs. 11.4 - 11.7) ---
% Mach number as a function of V_EAS [kts] (converts V_EAS -> V_TAS first)
calc_Mach = @(V_eas_kts) eas_to_tas(V_eas_kts * kts_to_ms) ./ a_snd;

% Prandtl-Glauert factor beta (Eq. 11.7)
calc_beta = @(V_eas_kts) sqrt(1 - calc_Mach(V_eas_kts).^2);

% DATCOM Cl_alpha as a function of V_EAS [kts] (Eq. 11.6)
calc_Cl_alpha = @(V_eas_kts) (2 * pi * AR) ./ ...
    (2 + sqrt(AR^2 .* (calc_beta(V_eas_kts) ./ kappa).^2 .* ...
    (1 + (tand(Lambda_c2)^2) ./ (calc_beta(V_eas_kts).^2)) + 4));

% Gust alleviation factor Kg as a function of Cl_alpha (Eq. 11.5)
calc_Kg = @(Cl_a) (0.88 .* ((2 * WS_SI) ./ (rho * c_bar .* Cl_a * g))) ./ ...
                  (5.3 + ((2 * WS_SI) ./ (rho * c_bar .* Cl_a * g)));

% Gust load factor increment delta_n (Eq. 11.4, requires V_EAS in knots)
calc_dn_gust = @(V_eas_kts, Ue) (calc_Kg(calc_Cl_alpha(V_eas_kts)) .* calc_Cl_alpha(V_eas_kts) .* Ue .* V_eas_kts) ./ (498 * WS_imp);

% FAR Part 25 Gust Velocities for altitude <= 20,000 ft (Table 11.1)
Ue_B = 66; % Rough air gust [ft/s]
Ue_C = 50; % Cruise gust [ft/s]
Ue_D = 25; % Dive gust [ft/s]

% --- 6. Find VB (Intersection of Stall Curve & Rough-Air Gust Line) ---
stall_n_func = @(V_eas_kts) (rho_SL * (V_eas_kts * kts_to_ms).^2 * Cl_max) / (2 * WS_SI);
gust_B_func  = @(V_eas_kts) 1 + calc_dn_gust(V_eas_kts, Ue_B);

VB_eas_kts = fzero(@(V_eas) stall_n_func(V_eas) - gust_B_func(V_eas), [VS_pos_eas_kts, VA_eas_kts * 1.5]);

% Check FAR requirement: VC >= VB + 43 knots for transport aircraft
if VC_eas_kts < VB_eas_kts + 43
    warning('VC (%.1f kts EAS) is less than VB + 43 kts (%.1f kts EAS). Consider increasing VC.', VC_eas_kts, VB_eas_kts + 43);
end

% Print airspeeds (EAS & TAS) and Cl_alpha at each design speed
fprintf('VB: %.1f kts EAS (%.1f kts TAS) | Cl_alpha: %.3f /rad\n', VB_eas_kts, eas_to_tas(VB_eas_kts), calc_Cl_alpha(VB_eas_kts));
fprintf('VC: %.1f kts EAS (%.1f kts TAS) | Cl_alpha: %.3f /rad\n', VC_eas_kts, eas_to_tas(VC_eas_kts), calc_Cl_alpha(VC_eas_kts));
fprintf('VD: %.1f kts EAS (%.1f kts TAS) | Cl_alpha: %.3f /rad\n', VD_eas_kts, eas_to_tas(VD_eas_kts), calc_Cl_alpha(VD_eas_kts));

% Gust load factors at VB, VC, VD
n_gust_B_pos = 1 + calc_dn_gust(VB_eas_kts, Ue_B);
n_gust_B_neg = 1 - calc_dn_gust(VB_eas_kts, Ue_B);

n_gust_C_pos = 1 + calc_dn_gust(VC_eas_kts, Ue_C);
n_gust_C_neg = 1 - calc_dn_gust(VC_eas_kts, Ue_C);

n_gust_D_pos = 1 + calc_dn_gust(VD_eas_kts, Ue_D);
n_gust_D_neg = 1 - calc_dn_gust(VD_eas_kts, Ue_D);

% --- 7. Construct Maneuver Envelope Curves ---
V_pos_stall = linspace(0, VA_eas_kts, 100);
n_pos_stall = stall_n_func(V_pos_stall);

V_pos_top   = [VA_eas_kts, VD_eas_kts];
n_pos_top   = [n_limit_pos, n_limit_pos];

V_neg_stall = linspace(0, VA_neg_eas_kts, 100);
n_neg_stall = (rho_SL * (V_neg_stall * kts_to_ms).^2 * Cl_min) / (2 * WS_SI);

V_neg_bot   = [VA_neg_eas_kts, VC_eas_kts, VD_eas_kts];
n_neg_bot   = [n_limit_neg, n_limit_neg, 0];

V_right     = [VD_eas_kts, VD_eas_kts];
n_right     = [0, n_limit_pos];

% --- 8. Plotting ---
figure('Color', 'w', 'Position', [100, 100, 900, 600]);
hold on; grid on;

% Plot Maneuver Envelope
p1 = plot(V_pos_stall, n_pos_stall, 'b-', 'LineWidth', 2);
plot(V_pos_top, n_pos_top, 'b-', 'LineWidth', 2);
plot(V_neg_stall, n_neg_stall, 'b-', 'LineWidth', 2);
plot(V_neg_bot, n_neg_bot, 'b-', 'LineWidth', 2);
plot(V_right, n_right, 'b-', 'LineWidth', 2);

% Plot Gust Envelope (radiating from (0,1) through VB, VC, VD)
V_gust_line = [0, VB_eas_kts, VC_eas_kts, VD_eas_kts];
p2 = plot(V_gust_line, [1, n_gust_B_pos, n_gust_C_pos, n_gust_D_pos], 'r--', 'LineWidth', 1.5);
plot(V_gust_line, [1, n_gust_B_neg, n_gust_C_neg, n_gust_D_neg], 'r--', 'LineWidth', 1.5);

% Plot individual gust rays from (0,1) for reference
plot([0, VC_eas_kts], [1, n_gust_C_pos], 'r:', 'LineWidth', 1);
plot([0, VD_eas_kts], [1, n_gust_D_pos], 'r:', 'LineWidth', 1);
plot([0, VC_eas_kts], [1, n_gust_C_neg], 'r:', 'LineWidth', 1);
plot([0, VD_eas_kts], [1, n_gust_D_neg], 'r:', 'LineWidth', 1);

% Reference lines (n = 0 and n = 1)
yline(0, 'k-', 'LineWidth', 0.8);
yline(1, 'k:', 'LineWidth', 0.8);

% Annotate Key Design Airspeeds
xline(VS_pos_eas_kts, '--k', sprintf('V_S = %.1f kts', VS_pos_eas_kts), 'LabelVerticalAlignment', 'bottom');
xline(VA_eas_kts,     '--k', sprintf('V_A = %.1f kts', VA_eas_kts),     'LabelVerticalAlignment', 'bottom');
xline(VB_eas_kts,     '--r', sprintf('V_B = %.1f kts', VB_eas_kts),     'LabelVerticalAlignment', 'top');
xline(VC_eas_kts,     '--k', sprintf('V_C = %.1f kts', VC_eas_kts),     'LabelVerticalAlignment', 'bottom');
xline(VD_eas_kts,     '--k', sprintf('V_D = %.1f kts', VD_eas_kts),     'LabelVerticalAlignment', 'bottom');

% Formatting
title('V-n Diagram (Maneuver and Gust Envelopes)');
xlabel('Equivalent Airspeed, V_{EAS} (knots)');
ylabel('Load Factor, n');
xlim([0, VD_eas_kts * 1.15]);
ylim([-2.0, max(n_limit_pos, n_gust_C_pos) + 0.8]);
legend([p1, p2], {'Maneuver Envelope', 'Gust Envelope (FAR 25)'}, 'Location', 'northwest');
hold off;