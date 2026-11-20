%% Payload-Range Diagram for M-BRAER Hybrid-Electric STOL Commuter
% Matches AE481 Notes (Algorithms 1-4 & Figure 3)
% Integrated with pipeline workspace outputs (selectedDesign, in, p)

if ~exist('selectedDesign', 'var') || ~selectedDesign.valid
    error('Run the master design workflow first so selectedDesign, in, and p are available.');
end

g0     = in.g0;
kgToLb = in.kgToLb;
lbToKg = 1 / kgToLb;

% --- 1. Extract Weights, Capacities & Propulsion Parameters ---
% Bookkeeping (Eq. 1-2): W_TO = W_OE + W_PL + W_F, where W_OE includes battery & crew
pilot_in_mEmpty      = true; % Set false if mEmpty_kg does NOT already include the 190+30 lb pilot
single_pilot_mass_kg = 220 * lbToKg; % 190 lb pilot + 30 lb bag per AE481 Notes Sec. 4

m_empty_excl_bat_kg  = selectedDesign.mass.mEmpty_kg;
if ~pilot_in_mEmpty
    m_empty_excl_bat_kg = m_empty_excl_bat_kg + single_pilot_mass_kg;
end
m_battery_kg   = selectedDesign.mass.mBattery_kg;
m_OE_kg        = m_empty_excl_bat_kg + m_battery_kg; % Operating Empty Weight (fixed on all missions)
MTOW_limit_kg  = selectedDesign.mass.mTO_kg;

% Payload limits (0 lb at ferry Point D up to max passenger + baggage payload)
pax_unit_lb      = 220; % 190 lb pax + 30 lb bag (Eq. 2)
m_payload_min_kg = 0;
m_payload_max_kg = in.mPeople_kg + in.mBaggage_kg;
W_PL_max_lb      = m_payload_max_kg * kgToLb;

% Volumetric Wing Tank Cap (60% structural box, 40% usable wet-wing volume)
wing_volume_m3      = 4.5954;
usable_vol_fraction = 0.40;
rho_jetA_kg_m3      = 804;
m_fuel_tank_max_kg  = wing_volume_m3 * usable_vol_fraction * rho_jetA_kg_m3;

% Battery Usable Energy (Eq. 4) — pull from struct if defined, otherwise set your sizing values
if isfield(in, 'E_bat_usable_J')
    E_bat_usable_J = in.E_bat_usable_J;
else
    % Fallback: update e_bat_Wh_kg and SOC window to match your sizing script
    e_bat_Wh_kg = 350;  % Pack-level specific energy [Wh/kg]
    SOC_0       = 1.00; % Initial SOC
    SOC_min     = 0.20; % Minimum reserve SOC (held for 5,000 ft engine-out emergency range)
    E_bat_usable_J = (SOC_0 - SOC_min) * m_battery_kg * e_bat_Wh_kg * 3600;
end

% Propulsion & Aerodynamic Parameters (from workspace)
LD_cruise    = in.LD_ref;
eta_prop     = in.etaProp;
eta_elec     = 0.90;                 % Battery-to-shaft efficiency (update from 'in' if named)
Cp_SI        = in.Cp_lb_hphr * (0.45359237 / (745.699872 * 3600)); % [kg / (W*s)]
K_breguet_nm = in.K_Breguet * eta_prop * LD_cruise / in.Cp_lb_hphr; % Breguet range factor [nmi]

% Mission Segment & Reserve Schedule (match your sizing mission!)
V_cruise_kts   = 225;   % Cruise speed [KTAS]
R_alt_nmi      = 100;   % IFR alternate distance [nmi] (14 CFR 91.167)
t_hold_hr      = 0.75;  % IFR 45-min cruise reserve [hr]
R_climb_nmi    = 20;    % Distance covered in climb [nmi] (set 0 if using pure cruise Breguet)
R_descent_nmi  = 20;    % Distance covered in descent [nmi]
W_frac_climb   = 0.992; % Engine fuel weight fraction during takeoff + climb (e.g. 50/50 split)
E_frac_climb   = 0.12;  % Fraction of usable battery energy spent in takeoff + climb

% --- 2. Mission Evaluator (Algorithms 1 & 2) ---
% Computes required fuel (block + IFR reserve), TOGW, fuel burned post-TOC, and feasibility
eval_mission = @(R_nmi, m_PL_kg) run_hybrid_mission( ...
    R_nmi, m_PL_kg, m_OE_kg, MTOW_limit_kg, m_fuel_tank_max_kg, E_bat_usable_J, ...
    K_breguet_nm, V_cruise_kts, eta_prop, eta_elec, LD_cruise, g0, ...
    R_alt_nmi, t_hold_hr, R_climb_nmi, R_descent_nmi, W_frac_climb, E_frac_climb);

% --- 3. Algorithm 3: Outer Payload-Range Envelope (A -> B -> C -> D) ---
m_PL_C_kg = MTOW_limit_kg - m_OE_kg - m_fuel_tank_max_kg;
payload_list_kg = unique([linspace(m_payload_max_kg, m_payload_min_kg, 40), ...
                          max(m_payload_min_kg, min(m_payload_max_kg, m_PL_C_kg))]);
payload_list_kg = sort(payload_list_kg, 'descend');

R_env_nmi       = zeros(size(payload_list_kg));
active_const    = strings(size(payload_list_kg));

for i = 1:length(payload_list_kg)
    P_kg = payload_list_kg(i);
    R_lo = 0;
    R_hi = 2500; % Upper bracket [nmi]
    
    [~, ~, ~, feas_0, ~] = eval_mission(R_lo, P_kg);
    if ~feas_0
        error('Payload %.1f lb exceeds MTOW even at 0 nmi range!', P_kg * kgToLb);
    end
    
    while (R_hi - R_lo) > 0.05
        R_mid = 0.5 * (R_lo + R_hi);
        [~, ~, ~, is_feas, ~] = eval_mission(R_mid, P_kg);
        if is_feas
            R_lo = R_mid;
        else
            R_hi = R_mid;
        end
    end
    R_env_nmi(i) = R_lo;
    [~, ~, ~, ~, active_const(i)] = eval_mission(R_lo + 0.1, P_kg);
end

% Identify Corners B (max range at max payload) and C (MTOW meets fuel cap)
R_PointB = R_env_nmi(1);
idx_C = find(active_const == "FUEL", 1, 'first');
if isempty(idx_C)
    idx_C = length(payload_list_kg);
end
R_PointC    = R_env_nmi(idx_C);
W_PL_C_lb   = payload_list_kg(idx_C) * kgToLb;
R_PointD    = R_env_nmi(end);

% --- 4. Algorithm 4: Electric-Range Boundary (E -> F) ---
% Boundary where zero fuel is burned after Top of Climb (cruise is 100% electric)
payload_elec_kg = linspace(m_payload_max_kg, m_payload_min_kg, 15);
R_elec_nmi      = zeros(size(payload_elec_kg));

for i = 1:length(payload_elec_kg)
    P_kg = payload_elec_kg(i);
    R_lo = 0;
    R_hi = R_PointB;
    while (R_hi - R_lo) > 0.05
        R_mid = 0.5 * (R_lo + R_hi);
        [~, ~, fuel_post_TOC, is_feas, ~] = eval_mission(R_mid, P_kg);
        if is_feas && (fuel_post_TOC <= 1e-4)
            R_lo = R_mid;
        else
            R_hi = R_mid;
        end
    end
    R_elec_nmi(i) = R_lo;
end

R_PointE = R_elec_nmi(1);   % Max electric range at full payload (Point E)
R_PointF = R_elec_nmi(end); % Max electric range at zero payload (Point F)

% --- 5. Plotting (Matching Figure 3 of AE481 Notes) ---
payload_env_lb  = payload_list_kg * kgToLb;
payload_elec_lb = payload_elec_kg * kgToLb;

figure('Color', 'w', 'Position', [120, 120, 960, 600]);
hold on; box on;

% Shaded Region I: Electric Cruise (bounded by x=0, A->E, E->F, y=0)
poly_Reg1_R  = [0, R_elec_nmi, 0];
poly_Reg1_PL = [W_PL_max_lb, payload_elec_lb, 0];
h_reg1 = fill(poly_Reg1_R, poly_Reg1_PL, [0.85, 0.93, 0.83], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.85, 'HandleVisibility', 'off');

% Shaded Region II: Hybrid / Engine-Extended Range (bounded by E->B->C->D->F)
poly_Reg2_R  = [R_elec_nmi, R_env_nmi, fliplr(R_elec_nmi)];
poly_Reg2_PL = [payload_elec_lb, payload_env_lb, zeros(size(R_elec_nmi))];
% Use clean polygon tracing E -> B -> C -> D -> F -> E:
poly_Reg2_R  = [R_PointE, R_env_nmi, R_PointF, fliplr(R_elec_nmi(2:end-1))];
poly_Reg2_PL = [W_PL_max_lb, payload_env_lb, 0, fliplr(payload_elec_lb(2:end-1))];
h_reg2 = fill(poly_Reg2_R, poly_Reg2_PL, [0.98, 0.89, 0.87], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.85, 'HandleVisibility', 'off');

% RFP Reference Vertical Lines
xline(200, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.1, 'HandleVisibility', 'off');
xline(400, '--', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.1, 'HandleVisibility', 'off');

% Plot Outer Envelope & Electric-Range Boundary
p_env  = plot([0, R_env_nmi], [W_PL_max_lb, payload_env_lb], 'k-', 'LineWidth', 2.2);
p_elec = plot(R_elec_nmi, payload_elec_lb, '-', 'Color', [0.42 0.65 0.25], 'LineWidth', 2.2);

% Plot Key Mission Markers (Design Mission & Max Electric Range Mission)
p_des  = plot(R_PointB, W_PL_max_lb, 'ko', 'MarkerFaceColor', [0.93 0.69 0.13], 'MarkerSize', 9, 'LineWidth', 1.2);
p_emax = plot(R_PointE, W_PL_max_lb, 'ks', 'MarkerFaceColor', [0.42 0.65 0.25], 'MarkerSize', 9, 'LineWidth', 1.2);

% Region & Constraint Labels (matching Figure 3)
text(R_PointE * 0.5, W_PL_max_lb * 0.48, {'Region I', 'electric cruise'}, ...
    'HorizontalAlignment', 'center', 'Color', [0.25 0.45 0.15], 'FontWeight', 'bold', 'FontSize', 10);
text(0.5 * (R_PointE + R_PointD), W_PL_max_lb * 0.48, ...
    {'Region II', 'battery used down to reserve,', 'engine extends the range'}, ...
    'HorizontalAlignment', 'center', 'Color', [0.55 0.25 0.20], 'FontWeight', 'bold', 'FontSize', 10);

text(0.5 * (R_PointE + R_PointB), W_PL_max_lb + 45, ...
    sprintf('max payload (%.0f pax)', W_PL_max_lb / pax_unit_lb), ...
    'HorizontalAlignment', 'center', 'FontSize', 9.5);
text(0.5 * (R_PointB + R_PointC) + 25, 0.5 * (W_PL_max_lb + W_PL_C_lb) + 55, ...
    sprintf('MTOW = %.0f lb', MTOW_limit_kg * kgToLb), ...
    'HorizontalAlignment', 'left', 'FontSize', 9.5);
text(0.5 * (R_PointC + R_PointD) - 25, W_PL_C_lb * 0.35, ...
    sprintf('fuel capacity\n%.0f lb', m_fuel_tank_max_kg * kgToLb), ...
    'HorizontalAlignment', 'right', 'FontSize', 9.5);

% RFP vertical line annotations
text(204, W_PL_max_lb * 0.08, {'RFP electric-range', 'objective (200 nmi)'}, ...
    'Color', [0.4 0.4 0.4], 'FontSize', 8.5);
text(404, W_PL_max_lb * 0.08, {'RFP design', 'range (400 nmi)'}, ...
    'Color', [0.4 0.4 0.4], 'FontSize', 8.5);

% Corner Letter Callouts (A, B, C, D, E, F)
text(8, W_PL_max_lb + 45, 'A', 'FontWeight', 'bold', 'FontSize', 11);
text(R_PointE - 12, W_PL_max_lb + 45, 'E', 'FontWeight', 'bold', 'FontSize', 11);
text(R_PointB + 8, W_PL_max_lb + 45, 'B', 'FontWeight', 'bold', 'FontSize', 11);
text(R_PointC + 10, W_PL_C_lb + 20, 'C', 'FontWeight', 'bold', 'FontSize', 11);
text(R_PointD + 10, 40, 'D', 'FontWeight', 'bold', 'FontSize', 11);
text(R_PointF + 8, 40, 'F', 'FontWeight', 'bold', 'FontSize', 11);

% Left Y-Axis (Payload in lb)
ylim_max = ceil((W_PL_max_lb * 1.18) / 200) * 200;
xlim_max = ceil((R_PointD * 1.12) / 100) * 100;
ylim([0, ylim_max]);
xlim([0, xlim_max]);
xlabel('Range (nmi)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Payload (lb)', 'FontSize', 11, 'FontWeight', 'bold');
grid on; set(gca, 'Layer', 'top');

% Right Y-Axis (Passengers with bags, 220 lb each)
yyaxis right;
ylim([0, ylim_max / pax_unit_lb]);
yticks(0:floor(W_PL_max_lb / pax_unit_lb));
ylabel('Passengers (with bags)', 'FontSize', 11, 'FontWeight', 'bold');
ax = gca; ax.YAxis(2).Color = 'k';
yyaxis left;

% Legend
legend([p_des, p_emax, p_env, p_elec], { ...
    sprintf('Design mission (%.0f nmi, %.0f pax)', R_PointB, W_PL_max_lb / pax_unit_lb), ...
    sprintf('Max electric range mission (%.0f nmi, %.0f pax)', R_PointE, W_PL_max_lb / pax_unit_lb), ...
    'Payload-range envelope', ...
    'Electric-range boundary'}, ...
    'Location', 'northoutside', 'NumColumns', 2, 'FontSize', 9.5);

title('M-BRAER Hybrid-Electric Payload-Range Diagram', 'FontSize', 12, 'FontWeight', 'bold');

% Summary Printout
fprintf('\n--- HYBRID-ELECTRIC PAYLOAD-RANGE SUMMARY (AE481 NOTES) ---\n');
fprintf('Point A (Zero Range, Max Payload)   : 0.0 nmi at %.0f lb\n', W_PL_max_lb);
fprintf('Point E (Max Electric Range)        : %.1f nmi at %.0f lb\n', R_PointE, W_PL_max_lb);
fprintf('Point F (Zero-Payload Electric)     : %.1f nmi at 0 lb\n', R_PointF);
fprintf('Point B (Design Corner, MTOW Hit)   : %.1f nmi at %.0f lb\n', R_PointB, W_PL_max_lb);
fprintf('Point C (MTOW + Volumetric Fuel Cap): %.1f nmi at %.0f lb\n', R_PointC, W_PL_C_lb);
fprintf('Point D (Max Ferry Range, 0 Pax)    : %.1f nmi at 0 lb\n', R_PointD);


%% Local Mission Evaluation Function (Algorithms 1 & 2)
function [m_TO_kg, m_F_total_kg, m_fuel_post_TOC_kg, is_feasible, violated] = run_hybrid_mission( ...
    R_nmi, m_PL_kg, m_OE_kg, MTOW_kg, m_F_cap_kg, E_bat_usable_J, ...
    K_breguet_nm, V_cruise_kts, eta_prop, eta_elec, LD_cruise, g0, ...
    R_alt_nmi, t_hold_hr, R_climb_nmi, R_descent_nmi, W_frac_climb, E_frac_climb)

    % Fixed-point iteration on fuel mass (Algorithm 2)
    m_F_guess = 500;
    for iter = 1:30
        m_TO = m_OE_kg + m_PL_kg + m_F_guess;
        
        % 1. Climb segment (battery + engine share)
        scale_cd = min(1.0, R_nmi / max(1e-3, R_climb_nmi + R_descent_nmi));
        m_after_climb = m_TO * (1 - (1 - W_frac_climb) * scale_cd);
        m_fuel_climb  = m_TO - m_after_climb;
        E_bat_rem     = E_bat_usable_J * (1 - E_frac_climb * scale_cd);
        
        % 2. Cruise distance
        R_cruise = max(0, R_nmi - (R_climb_nmi + R_descent_nmi) * scale_cd);
        
        % Electric range capability in cruise using remaining usable battery energy:
        % E_req = (W / (L/D)) * R / (eta_prop * eta_elec)
        W_cruise_N = m_after_climb * g0;
        R_elec_cruise_m   = (E_bat_rem * eta_prop * eta_elec * LD_cruise) / max(1.0, W_cruise_N);
        R_elec_cruise_nmi = R_elec_cruise_m / 1852.0;
        
        if R_cruise <= R_elec_cruise_nmi
            % Region I: Entire cruise flown on battery (zero fuel burned after TOC)
            m_fuel_cruise = 0;
            m_after_cruise = m_after_climb;
        else
            % Region II: Battery depletes to reserve SOC, engine flies remaining cruise
            R_fuel_cruise_nmi = R_cruise - R_elec_cruise_nmi;
            m_after_cruise = m_after_climb * exp(-R_fuel_cruise_nmi / K_breguet_nm);
            m_fuel_cruise  = m_after_climb - m_after_cruise;
        end
        
        % 3. Descent (assumed idle / electric windmilling -> ~0 fuel)
        m_land = m_after_cruise;
        
        % 4. IFR Reserve Fuel (14 CFR 91.167: 100 nmi alternate + 45 min at cruise speed)
        R_res_equiv_nmi = R_alt_nmi + V_cruise_kts * t_hold_hr;
        % Reserve fuel burned at end of mission (landing weight = m_OE + m_PL + m_F_res)
        m_zero_fuel = m_OE_kg + m_PL_kg;
        m_fuel_res  = m_zero_fuel * (exp(R_res_equiv_nmi / K_breguet_nm) - 1);
        
        m_F_new = m_fuel_climb + m_fuel_cruise + m_fuel_res;
        if abs(m_F_new - m_F_guess) < 0.05
            m_F_guess = m_F_new;
            break;
        end
        m_F_guess = m_F_new;
    end
    
    m_F_total_kg       = m_F_guess;
    m_TO_kg            = m_OE_kg + m_PL_kg + m_F_total_kg;
    m_fuel_post_TOC_kg = m_fuel_cruise;
    
    % Check constraints (Eq. 3)
    if m_TO_kg > MTOW_kg + 1e-3
        is_feasible = false;
        violated = "MTOW";
    elseif m_F_total_kg > m_F_cap_kg + 1e-3
        is_feasible = false;
        violated = "FUEL";
    else
        is_feasible = true;
        violated = "NONE";
    end
end