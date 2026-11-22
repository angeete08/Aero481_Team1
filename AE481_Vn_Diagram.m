%% Pipelined V-n diagram
if ~exist('selectedDesign', 'var') || ~selectedDesign.valid
    % Fallback if run independently
    if ~exist('in', 'var'), aircraft_inputs; end
    if ~exist('values_kg', 'var'), values_kg = Weight_Predict(in); end
    m_kg = values_kg(1);
    %m_kg = 6854.76;
    S = 88.64; % Default fallback area
else
    m_kg = selectedDesign.mass.mTO_kg;
    %m_kg = 6854.76;
    S = selectedDesign.S_m2;
end

fprintf('mass kg: %.2f\n', m_kg);

% --- 1. Aircraft & Atmospheric Parameters ---
W_lbs  = m_kg * in.kgToLb;          % Aircraft weight [lbs]
W_N    = m_kg * in.g0;              % Aircraft weight [N]

S_ft2  = S * 10.7639104167;         % Wing reference area [ft^2]
AR     = in.AR;                     % Wing aspect ratio
b      = sqrt(AR * S);              % Wing span [m]
c_bar  = S / b;                     % Mean geometric chord [m]
WS_SI  = W_N / S;                   % Wing loading [N/m^2]
WS_imp = W_lbs / S_ft2;             % Wing loading [lbs/ft^2]

% Altitude density using standard atmosphere matching pipeline
h_ft   = in.altitude_ft;            % [ft]
h_m    = h_ft * 0.3048;             % [m]
rho_SL = 1.225;                     % Sea-level air density [kg/m^3]
T0 = 288.15; p0 = 101325; L = 0.0065; R = 287.05287; g_acc = 9.80665;
Tisa = T0 - L * h_m;
p_alt = p0 * (Tisa/T0)^(g_acc/(R*L));
rho = p_alt / (R * Tisa);           % Flight altitude air density [kg/m^3]
sigma  = rho / rho_SL;              % Density ratio (Eq. 11.1)
a_snd  = sqrt(1.4 * R * Tisa);      % Speed of sound at flight altitude [m/s]
g      = g_acc;                     % Gravity [m/s^2]

% --- 2. Aerodynamic & Design Speed Inputs ---
Cl_max    = in.CLmax_clean;      % Clean max lift coefficient
Cl_min    = -1.4;        % Negative max lift coefficient
Lambda_c2 = 0;           % Mid-chord sweep angle [deg]
kappa     = 0.97;        % Empirical section lift-curve factor (Eq. 11.6)

% Unit conversion factors & Eq. (11.1) airspeed conversions
ms_to_kts  = 1.94384;
kts_to_ms  = 1 / ms_to_kts;
eas_to_tas = @(V_eas) V_eas ./ sqrt(sigma); % Eq. (11.1): V_TAS = V_EAS / sqrt(sigma)
tas_to_eas = @(V_tas) V_tas .* sqrt(sigma); % Eq. (11.1): V_EAS = V_TAS * sqrt(sigma)

% Design Airspeeds in EAS [knots]
VC_eas_kts = in.cruise_kt;       % Design cruise speed V_C [knots EAS] from 'in'
VD_eas_kts = 1.25 * VC_eas_kts;  % Design dive speed V_D [knots EAS]

% Aircraft under 50,000 lbs 
n_limit_pos = min(3.8, max(2.5, 2.1 + 24000 / (W_lbs + 10000)));
n_limit_neg = -n_limit_pos*0.4;      % FAR Part 25 negative limit load factor

fprintf('\n--- V-n DIAGRAM ANALYSIS (PIPELINE INTEGRATED) ---\n');
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
calc_Mach = @(V_eas_kts) eas_to_tas(V_eas_kts * kts_to_ms) ./ a_snd;
calc_beta = @(V_eas_kts) sqrt(1 - calc_Mach(V_eas_kts).^2);
calc_Cl_alpha = @(V_eas_kts) (2 * pi * AR) ./ ...
    (2 + sqrt(AR^2 .* (calc_beta(V_eas_kts) ./ kappa).^2 .* ...
    (1 + (tand(Lambda_c2)^2) ./ (calc_beta(V_eas_kts).^2)) + 4));

calc_Kg = @(Cl_a) (0.88 .* ((2 * WS_SI) ./ (rho * c_bar .* Cl_a * g))) ./ ...
                  (5.3 + ((2 * WS_SI) ./ (rho * c_bar .* Cl_a * g)));

calc_dn_gust = @(V_eas_kts, Ue) (calc_Kg(calc_Cl_alpha(V_eas_kts)) .* calc_Cl_alpha(V_eas_kts) .* Ue .* V_eas_kts) ./ (498 * WS_imp);

Ue_B = 66; % Rough air gust [ft/s]
Ue_C = 50; % Cruise gust [ft/s]
Ue_D = 25; % Dive gust [ft/s]

% --- 6. Find VB (Intersection of Stall Curve & Rough-Air Gust Line) ---
stall_n_func = @(V_eas_kts) (rho_SL * (V_eas_kts * kts_to_ms).^2 * Cl_max) / (2 * WS_SI);
gust_B_func  = @(V_eas_kts) 1 + calc_dn_gust(V_eas_kts, Ue_B);

VB_eas_kts = fzero(@(V_eas) stall_n_func(V_eas) - gust_B_func(V_eas), [VS_pos_eas_kts, VA_eas_kts * 1.5]);

if VC_eas_kts < VB_eas_kts + 43
    warning('VC (%.1f kts EAS) is less than VB + 43 kts (%.1f kts EAS). Consider increasing VC.', VC_eas_kts, VB_eas_kts + 43);
end

fprintf('VB: %.1f kts EAS (%.1f kts TAS) | Cl_alpha: %.3f /rad\n', VB_eas_kts, eas_to_tas(VB_eas_kts), calc_Cl_alpha(VB_eas_kts));
fprintf('VC: %.1f kts EAS (%.1f kts TAS) | Cl_alpha: %.3f /rad\n', VC_eas_kts, eas_to_tas(VC_eas_kts), calc_Cl_alpha(VC_eas_kts));
fprintf('VD: %.1f kts EAS (%.1f kts TAS) | Cl_alpha: %.3f /rad\n', VD_eas_kts, eas_to_tas(VD_eas_kts), calc_Cl_alpha(VD_eas_kts));

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

p1 = plot(V_pos_stall, n_pos_stall, 'b-', 'LineWidth', 2);
plot(V_pos_top, n_pos_top, 'b-', 'LineWidth', 2);
plot(V_neg_stall, n_neg_stall, 'b-', 'LineWidth', 2);
plot(V_neg_bot, n_neg_bot, 'b-', 'LineWidth', 2);
plot(V_right, n_right, 'b-', 'LineWidth', 2);

V_gust_line = [0, VB_eas_kts, VC_eas_kts, VD_eas_kts];
p2 = plot(V_gust_line, [1, n_gust_B_pos, n_gust_C_pos, n_gust_D_pos], 'r--', 'LineWidth', 1.5);
plot(V_gust_line, [1, n_gust_B_neg, n_gust_C_neg, n_gust_D_neg], 'r--', 'LineWidth', 1.5);

plot([0, VC_eas_kts], [1, n_gust_C_pos], 'r:', 'LineWidth', 1);
plot([0, VD_eas_kts], [1, n_gust_D_pos], 'r:', 'LineWidth', 1);
plot([0, VC_eas_kts], [1, n_gust_C_neg], 'r:', 'LineWidth', 1);
plot([0, VD_eas_kts], [1, n_gust_D_neg], 'r:', 'LineWidth', 1);

yline(0, 'k-', 'LineWidth', 0.8);
yline(1, 'k:', 'LineWidth', 0.8);

xline(VS_pos_eas_kts, '--k', sprintf('V_S = %.1f kts', VS_pos_eas_kts), 'LabelVerticalAlignment', 'bottom');
xline(VA_eas_kts,     '--k', sprintf('V_A = %.1f kts', VA_eas_kts),     'LabelVerticalAlignment', 'bottom');
xline(VB_eas_kts,     '--r', sprintf('V_B = %.1f kts', VB_eas_kts),     'LabelVerticalAlignment', 'top');
xline(VC_eas_kts,     '--k', sprintf('V_C = %.1f kts', VC_eas_kts),     'LabelVerticalAlignment', 'bottom');
xline(VD_eas_kts,     '--k', sprintf('V_D = %.1f kts', VD_eas_kts),     'LabelVerticalAlignment', 'bottom');

title('V-n Diagram (Maneuver and Gust Envelopes)');
xlabel('Equivalent Airspeed, V_{EAS} (knots)');
ylabel('Load Factor, n');
xlim([0, VD_eas_kts * 1.15]);
ylim([-2.0, max(n_limit_pos, n_gust_C_pos) + 0.8]);
legend([p1, p2], {'Maneuver Envelope', 'Gust Envelope (FAR 25)'}, 'Location', 'northwest');
hold off;