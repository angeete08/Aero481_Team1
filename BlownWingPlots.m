%% AE481 - BLOWN WING TAKEOFF PERFORMANCE & LIFT COEFFICIENT PLOT
% Generates a two-panel comparison matching the formatting of Slide 3 (Plot_image)
clear; clc; close all;

% --- 1. Load Baseline Design Parameters ---
if exist('Weight_Predict.m','file') ~= 2 || exist('Weight_Predict_PS.m','file') ~= 2
    error('Ensure Weight_Predict.m and Weight_Predict_PS.m are in this folder.');
end

baseline = Weight_Predict(400, 225, 10000, 18, 8, "electric", 210);
mTO_kg = baseline{'kg','GrossTakeoff'};
W_N = mTO_kg * 9.80665;

% Aircraft Configuration
WS_target_lbft2 = 20;            % Wing loading [lb/ft^2]
WS_target = WS_target_lbft2 * 47.880; % [N/m^2]
S_ref = W_N / WS_target;         % Reference wing area [m^2]
PW_installed_hp_lb = 0.15;       % Installed power [hp/lb]
PW_installed_W_N = PW_installed_hp_lb * 167.64; 
P_total_kW = (PW_installed_W_N * W_N) / 1000;

% Atmosphere & Physics Setup (SL ISA+18 deg F)
rho = 1.1839;                   % Density [kg/m^3]
sigma = 0.50;                   % Disk-area ratio Ad/S
eta_a = 0.80;                   % Propeller loss factor
AR = 7.0;                       % Aspect ratio
e_clean = 0.426;                % Oswald efficiency factor
CD0_TO = 0.10;                  % Gear + Flap drag

% Speeds & Flap Settings
V_kt_vec = linspace(15, 70, 200);
V_ms_vec = V_kt_vec * 0.514444;

% --- 2. Calculate Powered Lift Envelope & Level-Flight CL ---
CL_level_15 = (15 * 47.880) ./ (0.5 * rho * V_ms_vec.^2);
CL_level_20 = (20 * 47.880) ./ (0.5 * rho * V_ms_vec.^2);
CL_level_25 = (25 * 47.880) ./ (0.5 * rho * V_ms_vec.^2);

% Blown Lift for Takeoff (delta_F = 40 deg, eta_F = 0.912 -> delta_j = 36.48 deg)
delta_F = 40;
eta_F = 0.912;
delta_j = eta_F * delta_F;

CL_blown_100 = zeros(size(V_ms_vec));
CL_blown_70  = zeros(size(V_ms_vec));
alpha_allow_vec = zeros(size(V_ms_vec));

for i = 1:length(V_ms_vec)
    V = V_ms_vec(i);
    q = 0.5 * rho * V^2;
    
    % 100% Power Calculations
    P_sh_S = (PW_installed_W_N * WS_target); % Shaft power per S
    cubic_coeff = [1, V, -V^2, -(V^3 + (4*eta_a*P_sh_S)/(rho*sigma))];
    r_roots = roots(cubic_coeff);
    Vj = max(r_roots(imag(r_roots)==0 & r_roots > V));
    
    Tc_prime = sigma * ((Vj/V)^2 - 1);
    dCj = Tc_prime * (1 + V/Vj);
    
    alpha_stall = min(16 + 1.5*Tc_prime, 22);
    alpha_allow = alpha_stall - 8; % 8 deg takeoff margin
    alpha_allow_vec(i) = alpha_allow;
    
    % Lift Fit Terms (Appendix A1)
    CL_u = 0.1856 + 0.0334*delta_F + 0.0667*alpha_allow;
    A1 = 0.0121*delta_j - 0.0085*alpha_allow + 0.0426;
    A2 = 0.0140*delta_j + 0.0226*alpha_allow + 0.1407;
    dCL_blown = A1*sqrt(dCj) + A2*dCj;
    
    CL_blown_100(i) = CL_u + dCL_blown;
    
    % 70% Power Calculations
    cubic_coeff_70 = [1, V, -V^2, -(V^3 + (4*eta_a*0.7*P_sh_S)/(rho*sigma))];
    r_roots_70 = roots(cubic_coeff_70);
    Vj_70 = max(r_roots_70(imag(r_roots_70)==0 & r_roots_70 > V));
    Tc_prime_70 = sigma * ((Vj_70/V)^2 - 1);
    dCj_70 = Tc_prime_70 * (1 + V/Vj_70);
    alpha_stall_70 = min(16 + 1.5*Tc_prime_70, 22);
    alpha_allow_70 = alpha_stall_70 - 8;
    
    CL_u_70 = 0.1856 + 0.0334*delta_F + 0.0667*alpha_allow_70;
    A1_70 = 0.0121*delta_j - 0.0085*alpha_allow_70 + 0.0426;
    A2_70 = 0.0140*delta_j + 0.0226*alpha_allow_70 + 0.1407;
    CL_blown_70(i) = CL_u_70 + A1_70*sqrt(dCj_70) + A2_70*dCj_70;
end

% --- 3. Time-March Takeoff Distance (Left Panel Data) ---
flaps_sweep = [20, 30, 40];
speeds_TO = 15:1:60;
s_TO_dist = zeros(length(flaps_sweep), length(speeds_TO));

for f_idx = 1:length(flaps_sweep)
    df = flaps_sweep(f_idx);
    ef = 0.912;
    dj = ef * df;
    for s_idx = 1:length(speeds_TO)
        V_target = speeds_TO(s_idx) * 0.514444;
        
        % Approximate 3-phase takeoff distance integration
        % Phase 1: Static roll to 5 m/s
        T0 = (2*rho*S_ref*sigma)^(1/3) * (eta_a * P_total_kW * 1000)^(2/3);
        a0 = 9.80665 * (T0/W_N - 0.04);
        s1 = 25 / (2*a0);
        
        % Phase 2 & 3: Roll and Climb to 50 ft
        dt = 0.02; V_sim = 5; s_sim = s1; h_sim = 0; gamma_sim = 0;
        while h_sim < 15.24 && s_sim < 1500
            q_sim = 0.5 * rho * V_sim^2;
            cubic_sim = [1, V_sim, -V_sim^2, -(V_sim^3 + (4*eta_a*P_sh_S)/(rho*sigma))];
            r_sim = roots(cubic_sim);
            Vj_sim = max(r_sim(imag(r_sim)==0 & r_sim > V_sim));
            Tc_sim = sigma * ((Vj_sim/V_sim)^2 - 1);
            dCj_sim = Tc_sim * (1 + V_sim/Vj_sim);
            
            a_allow = min(16 + 1.5*Tc_sim, 22) - 8;
            CL_u_s = 0.1856 + 0.0334*df + 0.0667*a_allow;
            A1_s = 0.0121*dj - 0.0085*a_allow + 0.0426;
            A2_s = 0.0140*dj + 0.0226*a_allow + 0.1407;
            CL_sim = CL_u_s + A1_s*sqrt(dCj_sim) + A2_s*dCj_sim;
            
            L_sim = q_sim * S_ref * CL_sim;
            CX_sim = CD0_TO + (CL_sim^2)/(pi*AR*e_clean + 2*dCj_sim) - Tc_sim;
            X_sim = q_sim * S_ref * CX_sim;
            
            if h_sim == 0
                if L_sim >= W_N
                    h_sim = h_sim + 0.01; % Liftoff
                else
                    dVdt = (-X_sim - 0.04*(W_N - L_sim)) / (W_N/9.80665);
                    V_sim = V_sim + dVdt*dt;
                    s_sim = s_sim + V_sim*dt;
                end
            else
                gamma_sim = gamma_sim + ((L_sim - W_N*cos(gamma_sim))/(W_N/9.80665*V_sim))*dt;
                dVdt = (-X_sim - W_N*sin(gamma_sim)) / (W_N/9.80665);
                V_sim = V_sim + dVdt*dt;
                s_sim = s_sim + V_sim*cos(gamma_sim)*dt;
                h_sim = h_sim + V_sim*sin(gamma_sim)*dt;
            end
        end
        s_TO_dist(f_idx, s_idx) = s_sim * 3.28084; % Convert to ft
    end
end

% --- 4. Plotting (Formatting identical to Plot_image) ---
figure('Color','w','Position',[100 100 1200 550]);

% Left Panel: Takeoff Distance vs Liftoff Speed
subplot(1,2,1); hold on; grid on; box on;
plot(speeds_TO, s_TO_dist(1,:), 'LineWidth', 2.0, 'Color', [0.4 0.4 0.4], 'DisplayName', 'Flaps 20^\circ');
plot(speeds_TO, s_TO_dist(2,:), 'LineWidth', 2.0, 'Color', [0.0 0.45 0.55], 'DisplayName', 'Flaps 30^\circ');
plot(speeds_TO, s_TO_dist(3,:), 'LineWidth', 2.5, 'Color', [0.1 0.15 0.3], 'DisplayName', 'Flaps 40^\circ (Full Blown)');
yline(300, '--', 'LineWidth', 2.0, 'Color', [0.85 0.55 0.0], 'DisplayName', 'RFP: 300 ft over 50 ft');

xlabel('Liftoff speed (no flare) [kt]', 'FontSize', 11);
ylabel('Takeoff distance over 50 ft [ft]', 'FontSize', 11);
title('Takeoff: ground roll + rotation + climb', 'FontSize', 12, 'FontWeight', 'bold');
ylim([0 1200]); xlim([15 60]);
legend('Location', 'northwest', 'FontSize', 9);

% Right Panel: Level-Flight CL vs Speeds & Blown Lift Envelope
subplot(1,2,2); hold on; grid on; box on;
plot(V_kt_vec, CL_level_15, 'LineWidth', 2.0, 'Color', [0.25 0.5 0.55], 'DisplayName', 'W/S = 15 lb/ft^2');
plot(V_kt_vec, CL_level_20, 'LineWidth', 2.5, 'Color', [0.1 0.15 0.3], 'DisplayName', 'W/S = 20 lb/ft^2');
plot(V_kt_vec, CL_level_25, 'LineWidth', 2.0, 'Color', [0.4 0.4 0.4], 'DisplayName', 'W/S = 25 lb/ft^2');

plot(V_kt_vec, CL_blown_100, 'LineWidth', 2.5, 'Color', [0.7 0.1 0.1], 'DisplayName', 'Max Blown C_L (100% Power)');
plot(V_kt_vec, CL_blown_70, '--', 'LineWidth', 1.8, 'Color', [0.85 0.35 0.1], 'DisplayName', 'Blown C_L (70% Power)');

yline(2.3, ':', 'LineWidth', 1.8, 'Color', [0.5 0.5 0.5], 'DisplayName', 'Flapped, unblown C_{L,max} \approx 2.3');

xlabel('Speed [kt]', 'FontSize', 11);
ylabel('Level-flight C_L = (W/S)/q  &  Blown C_L', 'FontSize', 11);
title('Blown Takeoff Lift Coefficient Capability', 'FontSize', 12, 'FontWeight', 'bold');
ylim([0 16]); xlim([15 70]);
legend('Location', 'northeast', 'FontSize', 9);

sgtitle('AE481 Hybrid eSTOL: Blown Wing Takeoff Performance Requirements', 'FontSize', 14, 'FontWeight', 'bold');