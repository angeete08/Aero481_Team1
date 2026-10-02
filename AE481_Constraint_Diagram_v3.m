%% AE481 Assignment 4 - Parametric SI Sizing
% M-BRAER Hybrid-Electric STOL Commuter
%
% SI-UNIT VERSION
%
% Internal units:
%   mass                kg
%   weight              N
%   length              m
%   area                m^2
%   speed               m/s
%   density             kg/m^3
%   power               W
%   wing loading W/S    N/m^2
%   power-to-weight P/W W/N
%
% For the workbook, the selected point is also reported as:
%   W/S  [kg/m^2]
%   W/P  [kg/kW]
%
% Performance - Overall and Performance - Aerodynamics are OUTPUTS.
% The workbook is NOT read by this script.
%
% ELECTRIC-RANGE UPDATE:
% The feasible W/S region now also requires the existing battery to achieve
% the selected 210-nmi electric cruise target at the 225-kt threshold speed.
% This is a cruise-energy screen; battery/motor continuous-power limits and
% full mission reserves still require separate verification.
%
% NOTE ON TAKEOFF & LANDING:
% Both takeoff and landing are modeled with physics-based 3-phase methods:
%   Takeoff:  1) ground roll, 2) circular transition, 3) steady climb to 50 ft.
%   Landing:  1) steady approach, 2) circular flare transition, 3) braking ground roll.
%
% No toolbox is required.

clear;
clc;
close all;

%% ========================================================================
% 1. UPDATED EXCEL DATA - MANUALLY ENTERED IN SI UNITS
% ========================================================================

g0 = 9.80665;                            % [m/s^2]

% Weights - Overall, converted from your updated workbook
m_TO_kg              = 5637.7;       % takeoff mass [kg]
m_empty_kg           = 3449;         % empty mass [kg]
m_fuel_kg            = 439.23;       % fuel mass [kg]
m_crew_pax_kg        = 689.46;       % crew + passengers [kg]
m_baggage_kg         = 108.86;       % baggage [kg]
m_electric_system_kg = 123.5;        % electric propulsion system [kg]
m_battery_kg         = 951.23;       % battery mass [kg]

W_TO_N = m_TO_kg*g0;                     % takeoff weight force [N]

% Current mission values
range_design_km   = 400*1.852;           % 740.8 km
range_electric_km = 210*1.852;           % 388.92 km

%% ========================================================================
% 2. RFP REQUIREMENTS - SI UNITS
% ========================================================================

Vcruise_threshold_ms = 225*0.514444;     % [m/s]
Vcruise_objective_ms = 250*0.514444;     % [m/s]

s_TO_req_m = 300*0.3048;                 % 91.44 m
s_L_req_m  = 300*0.3048;                 % 91.44 m

ROC_RFP_ms = 1500*0.00508;               % 7.62 m/s

h_obstacle_m = 50*0.3048;                % 15.24 m

% Required field condition: sea level, ISA + 18 F = ISA + 10 C
h_field_m = 0;
dT_field_C = 10;

% RFP 5000-ft hot-day reporting condition
h_5000_m = 5000*0.3048;                  % 1524 m
dT_5000_C = 10;

%% ========================================================================
% 3. SELECTED DESIGN ASSUMPTIONS
% ========================================================================

%% 3A. General configuration
AR = 10.0;                               % aspect ratio (Twin Otter)

h_cruise_m = 10000*0.3048;              % 3048 m preliminary cruise altitude
dT_cruise_C = 0;

% Weight fractions
beta_cruise  = 0.97;                     % W_cruise / W_TO
beta_landing = 1.00;                     % conservative max landing weight
beta_climb   = 1.00;

%% 3B. Preliminary high-lift / powered-lift selections
CLmax_clean = 1.80;
CLmax_TO    = 3;
CLmax_L     = 4;

%% 3C. Preliminary parasite-drag method
wet_c_SI = -0.84050366;
wet_d    =  0.8099;

Cf_equiv = 0.0030;

% Preliminary drag increments
dCD0_TO_flaps = 0.015;    % VERIFY
dCD0_L_flaps  = 0.065;
dCD0_gear     = 0.020;

% Span efficiencies
e_clean = 0.825;       % VERIFY
e_TO    = 0.775;
e_L     = 0.725;

%% 3D. Propeller efficiencies / available power
eta_TO     = 0.75;
eta_climb  = 0.80;
eta_cruise = 0.85;
eta_balked = 0.75;

powerFrac_TO       = 1.00;
powerFrac_climb    = 1.00;
powerFrac_cruise   = 0.80;
powerFrac_balked   = 1.00;

powerFrac_maneuver = 0.80;                % continuous power at maneuver condition
powerFrac_ceiling  = 0.70;                % installed TO-rated power available at 25 kft

%% 3E. Critical Loss of Thrust (CLoT)
CLoT_power_remaining = 0.50;

dCD0_CLoT = 0.010;     % Verify
e_CLoT    = 0.75;   

%% 3F. FAA/EASA Level-3 climb requirements
isHighSpeed = false; 

G_initial = 0.04;

if isHighSpeed
    G_CLoT = 0.02;  % If we enter into a high speed requirement
else
    G_CLoT = 0.01;
end

G_balked = 0.03;

% Stall speed safety margins
ks_takeoff = 1.10;
ks_initial = 1.20;
ks_CLoT    = 1.20;
ks_balked  = 1.30;
ks_landing = 1.30;

%% 3G. Physics-based extreme-STOL takeoff assumptions
mu_roll            = 0.02;               % rolling-friction coefficient, preliminary
k_rotation         = 1.10;               % V_R / V_s
k_transition       = 1.15;               % V_tr / V_s
gamma_takeoff_deg  = 15;                 % target climb flight-path angle [deg]
n_transition       = 1.50;              % transition load factor [VERIFY]

%% 3H. Extreme-STOL landing assumptions
gamma_approach_deg = 15;
k_touchdown        = 1.15;               % V_TD / V_s
stop_decel_g       = 0.65;               % braking deceleration [g]
n_flare            = 1.15;               % flare load factor [VERIFY]

%% 3I. Battery assumptions for battery-only emergency range
battery_specific_energy_Whkg = 700;
battery_usable_fraction      = 0.80;

eta_battery    = 0.95;
eta_wiring     = 0.98;
eta_controller = 0.98;
eta_motor      = 0.96;
eta_prop_elec  = 0.85;

%% 3J. Maneuver / bank-angle design requirement
use_maneuver_constraint = true;
phi_maneuver_deg  = 60;                   % max-bank design target [deg]
n_maneuver        = 1/cosd(phi_maneuver_deg);
h_maneuver_m      = h_cruise_m;          % [m], TEAM DESIGN ASSUMPTION
dT_maneuver_C     = dT_cruise_C;
V_maneuver_ms     = Vcruise_threshold_ms;% [m/s], TEAM DESIGN ASSUMPTION
beta_maneuver     = beta_cruise;
eta_maneuver      = eta_cruise;

%% 3K. Service-ceiling design requirement
use_ceiling_constraint = true;
h_ceiling_m       = 25000*0.3048;         % 7620 m [TEAM DESIGN ASSUMPTION]
dT_ceiling_C      = 0;                    % ISA [TEAM DESIGN ASSUMPTION]
ROC_ceiling_ms    = 100*0.00508;          % 100 ft/min = 0.508 m/s 
beta_ceiling      = beta_cruise;          % conservative current-weight fraction
eta_ceiling       = eta_cruise;
ks_ceiling         = 1.10;                % minimum 10% stall-speed margin

%% 3L. Constraint-chart settings
WS_Nm2 = linspace(250,4800,1500);         % true SI W/S [N/m^2]

WS_design_fraction = 0.95; 
PW_design_margin   = 1.05;

use_stall_constraint = false;             
Vstall_limit_ms = 50*0.514444;            

%% ========================================================================
% 4. ATMOSPHERE
% ========================================================================

rho_SL       = atmDensitySI(0,0);
rho_field    = atmDensitySI(h_field_m,dT_field_C);
rho_5000     = atmDensitySI(h_5000_m,dT_5000_C);
rho_cruise   = atmDensitySI(h_cruise_m,dT_cruise_C);
rho_maneuver = atmDensitySI(h_maneuver_m,dT_maneuver_C);
rho_ceiling  = atmDensitySI(h_ceiling_m,dT_ceiling_C);

sigma_field   = rho_field/rho_SL;
sigma_5000    = rho_5000/rho_SL;
sigma_ceiling = rho_ceiling/rho_SL;

%% ========================================================================
% 5. PARAMETRIC DRAG POLARS
% ========================================================================

Swet_m2 = 10^(wet_c_SI + wet_d*log10(m_TO_kg));  
S_grid_m2 = W_TO_N./WS_Nm2;
f_parasite_m2 = Cf_equiv*Swet_m2;

CD0_clean = f_parasite_m2./S_grid_m2;
CD0_TO_GU = CD0_clean + dCD0_TO_flaps;
CD0_TO_GD = CD0_clean + dCD0_TO_flaps + dCD0_gear;
CD0_L_GD  = CD0_clean + dCD0_L_flaps  + dCD0_gear;

%% ========================================================================
% 6. W/S LIMITS
% ========================================================================

% 6A. Explicit extreme-STOL physics-based landing limit
sLanding_grid_m = landingDistanceExtremeSTOL_SI( ...
    WS_Nm2,beta_landing,rho_field,CLmax_L,gamma_approach_deg, ...
    k_touchdown,stop_decel_g,h_obstacle_m,n_flare);

landingOK = sLanding_grid_m <= s_L_req_m;

if ~any(landingOK)
    error(['No point in the selected W/S range satisfies the 300-ft ', ...
           'landing model. Revisit the STOL assumptions.']);
end

WS_landing_max_Nm2 = max(WS_Nm2(landingOK));

% 6B. Optional stall-speed limit
if use_stall_constraint
    WS_stall_max_Nm2 = stallWsLimitSI( ...
        Vstall_limit_ms,rho_field,CLmax_L,beta_landing);
else
    WS_stall_max_Nm2 = inf;
end

WS_vertical_limit_Nm2 = min(WS_landing_max_Nm2,WS_stall_max_Nm2);

%% ========================================================================
% 7. P/W CONSTRAINTS - TRUE SI, W/N
% ========================================================================

% 7A. RFP takeoff field length - PHYSICS-BASED three-phase model
K_TO_constraint = 1/(pi*AR*e_TO);
CL_ground_TO = CLmax_TO/k_rotation^2;
CD_ground_TO = CD0_TO_GD + K_TO_constraint*CL_ground_TO^2;

[PW_takeoff_WN,takeoffGeometryOK, ...
    sGround_req_grid_m,sTransition_grid_m,sClimb_grid_m, ...
    hTransition_grid_m] = physicsTakeoffPW_SI( ...
    WS_Nm2,s_TO_req_m,h_obstacle_m,rho_field,CLmax_TO, ...
    CL_ground_TO,CD_ground_TO,mu_roll,eta_TO,powerFrac_TO, ...
    k_rotation,k_transition,n_transition,gamma_takeoff_deg);

% 7B. RFP 1500-ft/min initial climb
PW_RFP_ROC_WN = climbPWfromROC_SI( ...
    WS_Nm2,beta_climb,rho_field,eta_climb, ...
    CD0_TO_GD,e_TO,AR,CLmax_TO,ks_initial,ROC_RFP_ms, ...
    1.0,powerFrac_climb);

% 7C. FAA/EASA Level-3 initial climb
PW_initial_WN = climbPWfromGradient_SI( ...
    WS_Nm2,beta_climb,rho_field,eta_climb, ...
    CD0_TO_GD,e_TO,AR,CLmax_TO,ks_initial,G_initial, ...
    1.0,powerFrac_climb);

% 7D. FAA/EASA CLoT climb
PW_CLoT_WN = climbPWfromGradient_SI( ...
    WS_Nm2,beta_climb,rho_field,eta_climb, ...
    CD0_TO_GU+dCD0_CLoT,e_CLoT,AR,CLmax_TO,ks_CLoT,G_CLoT, ...
    CLoT_power_remaining,powerFrac_climb);

% 7E. FAA/EASA balked landing
PW_balked_WN = climbPWfromGradient_SI( ...
    WS_Nm2,beta_landing,rho_field,eta_balked, ...
    CD0_L_GD,e_L,AR,CLmax_L,ks_balked,G_balked, ...
    1.0,powerFrac_balked);

% 7F. RFP 225-knot threshold cruise
% Metabook Sec. 4.3, Eqs. (4.7)-(4.9): estimate ONE reference
% clean drag polar for the preliminary cruise constraint sweep.
% The old CD0_clean array is proportional to W/S because Swet is fixed
% while S_grid = W_TO/(W/S). That cancels the 1/(W/S) parasite term.
%
% Preliminary reference area: use the script's existing 95%-of-limit
% sizing rule. This is a design assumption, not a metabook-prescribed
% wing area. Replace S_cruise_ref_m2 with the actual baseline wing area
% when available, then update this polar as the design is refined.
WS_cruise_ref_Nm2 = WS_design_fraction*WS_vertical_limit_Nm2;
S_cruise_ref_m2 = W_TO_N/WS_cruise_ref_Nm2;
CD0_cruise_ref = Cf_equiv*Swet_m2/S_cruise_ref_m2;

% Fixed reference CD0 for BOTH cruise curves; retain the existing
% cruise weight fraction, propeller efficiency and power availability.
PW_cruise_225_WN = cruisePW_SI( ...
    WS_Nm2,beta_cruise,rho_cruise,eta_cruise, ...
    CD0_cruise_ref,e_clean,AR,Vcruise_threshold_ms,powerFrac_cruise);

% 7G. RFP 250-knot objective cruise
PW_cruise_250_WN = cruisePW_SI( ...
    WS_Nm2,beta_cruise,rho_cruise,eta_cruise, ...
    CD0_cruise_ref,e_clean,AR,Vcruise_objective_ms,powerFrac_cruise);

% 7H. Maximum-bank / sustained-maneuver constraint
if use_maneuver_constraint
    [PW_maneuver_WN,maneuverLiftOK] = maneuverPW_SI( ...
        WS_Nm2,beta_maneuver,rho_maneuver,eta_maneuver, ...
        CD0_clean,e_clean,AR,CLmax_clean,V_maneuver_ms, ...
        n_maneuver,powerFrac_maneuver);
else
    PW_maneuver_WN = zeros(size(WS_Nm2));
    maneuverLiftOK = true(size(WS_Nm2));
end

% 7I. Service-ceiling constraint
if use_ceiling_constraint
    [PW_ceiling_WN,V_ceiling_best_grid_ms,CL_ceiling_grid] = ...
        serviceCeilingPW_SI( ...
        WS_Nm2,beta_ceiling,rho_ceiling,eta_ceiling, ...
        CD0_clean,e_clean,AR,CLmax_clean,ks_ceiling,ROC_ceiling_ms, ...
        powerFrac_ceiling);
else
    PW_ceiling_WN = zeros(size(WS_Nm2));
    V_ceiling_best_grid_ms = nan(size(WS_Nm2));
    CL_ceiling_grid = nan(size(WS_Nm2));
end

% 7J. REQUIRED ELECTRIC CRUISE RANGE - 210 nmi AT 225 kt
beta_electric = 1.00;
V_electric_ms = Vcruise_threshold_ms;
q_electric = 0.5*rho_cruise*V_electric_ms^2;
K_electric = 1/(pi*AR*e_clean);

CL_electric = beta_electric.*WS_Nm2./q_electric;
CD_electric = CD0_clean + K_electric.*CL_electric.^2;
D_electric_N = q_electric.*S_grid_m2.*CD_electric;

eta_electric_total_range = ...
    eta_battery*eta_wiring*eta_controller*eta_motor*eta_prop_elec;

E_battery_usable_J = ...
    m_battery_kg*battery_specific_energy_Whkg*3600* ...
    battery_usable_fraction;

E_propulsive_range_J = ...
    E_battery_usable_J*eta_electric_total_range;

R_electric_grid_m = E_propulsive_range_J./D_electric_N;
R_electric_required_m = range_electric_km*1000;
electricRangeOK = R_electric_grid_m >= R_electric_required_m;

%% ========================================================================
% 8. FEASIBLE REGION AND PRELIMINARY DESIGN POINT
% ========================================================================

thresholdMatrix = [ ...
    PW_takeoff_WN;
    PW_RFP_ROC_WN;
    PW_initial_WN;
    PW_CLoT_WN;
    PW_balked_WN;
    PW_cruise_225_WN];

thresholdNames = { ...
    'Physics takeoff <= 91.44 m';
    'RFP initial climb >= 7.62 m/s';
    'FAA/EASA initial climb >= 4%';
    'FAA/EASA CLoT climb';
    'FAA/EASA balked landing >= 3%';
    'RFP cruise >= 115.75 m/s'};

if use_maneuver_constraint
    thresholdMatrix = [thresholdMatrix; PW_maneuver_WN];
    thresholdNames{end+1,1} = sprintf('Sustained %.0f-deg bank at %.0f kt', ...
        phi_maneuver_deg,V_maneuver_ms/0.514444);
end

if use_ceiling_constraint
    thresholdMatrix = [thresholdMatrix; PW_ceiling_WN];
    thresholdNames{end+1,1} = sprintf('Service ceiling %.0f ft / %.0f fpm', ...
        h_ceiling_m/0.3048,ROC_ceiling_ms/0.00508);
end

[PW_envelope_WN,~] = max(thresholdMatrix,[],1);

verticalOK = WS_Nm2 <= WS_vertical_limit_Nm2;
combinedFeasible = verticalOK & electricRangeOK & takeoffGeometryOK & maneuverLiftOK;

if any(combinedFeasible)
    WS_feasible_Nm2 = WS_Nm2(combinedFeasible);
    WS_low_Nm2 = min(WS_feasible_Nm2);
    WS_high_Nm2 = max(WS_feasible_Nm2);
    WS_target_Nm2 = WS_low_Nm2 + ...
        WS_design_fraction*(WS_high_Nm2 - WS_low_Nm2);
    [~,iClosestWS] = min(abs(WS_feasible_Nm2 - WS_target_Nm2));
    WS_design_Nm2 = WS_feasible_Nm2(iClosestWS);
    combinedFeasibleExists = true;
else
    warning(['No W/S in the investigated range simultaneously satisfies ', ...
        'the physics takeoff geometry, landing/STOL limit, electric-range ', ...
        'screen, and selected maneuver-lift requirement. The selected ', ...
        'point below is diagnostic only.']);
    WS_design_Nm2 = WS_design_fraction*WS_vertical_limit_Nm2;
    combinedFeasibleExists = false;
end

PW_required_WN = interp1( ...
    WS_Nm2,PW_envelope_WN,WS_design_Nm2,'linear');

PW_design_WN = PW_design_margin*PW_required_WN;

R_electric_selected_m = interp1( ...
    WS_Nm2,R_electric_grid_m,WS_design_Nm2,'linear');
R_electric_selected_km = R_electric_selected_m/1000;
electricRangeMargin_km = R_electric_selected_km - range_electric_km;
electricRangePassSelected = ...
    R_electric_selected_m >= R_electric_required_m;

PW_at_design = zeros(size(thresholdMatrix,1),1);

for i = 1:size(thresholdMatrix,1)
    PW_at_design(i) = interp1( ...
        WS_Nm2,thresholdMatrix(i,:),WS_design_Nm2,'linear');
end

[~,idxGoverning] = max(PW_at_design);
governingConstraint = thresholdNames{idxGoverning};

S_design_m2 = W_TO_N/WS_design_Nm2;

P_installed_W  = PW_design_WN*W_TO_N;
P_installed_kW = P_installed_W/1000;

massLoading_design_kgm2 = WS_design_Nm2/g0;

specificPower_design_kWkg = PW_design_WN*g0/1000;
WP_design_kgkW = 1/specificPower_design_kWkg;

%% ========================================================================
% 9. PERFORMANCE - AERODYNAMICS OUTPUTS
% ========================================================================

CD0_clean_sel = f_parasite_m2/S_design_m2;

CD0_TO_GU_sel = CD0_clean_sel + dCD0_TO_flaps;
CD0_TO_GD_sel = CD0_clean_sel + dCD0_TO_flaps + dCD0_gear;
CD0_L_GD_sel  = CD0_clean_sel + dCD0_L_flaps  + dCD0_gear;

K_clean = 1/(pi*AR*e_clean);
K_TO    = 1/(pi*AR*e_TO);
K_L     = 1/(pi*AR*e_L);

% Takeoff
Vs_TO_ms = stallSpeedSI( ...
    WS_design_Nm2,beta_climb,rho_field,CLmax_TO);

V_TO_ms = ks_takeoff*Vs_TO_ms;

CL_TO_rep = CLmax_TO/ks_takeoff^2;
CD_TO_rep = CD0_TO_GD_sel + K_TO*CL_TO_rep^2;
LD_TO = CL_TO_rep/CD_TO_rep;

% Climb
V_climb_ms = ks_initial*Vs_TO_ms;

CL_climb = CLmax_TO/ks_initial^2;
CD_climb = CD0_TO_GU_sel + K_TO*CL_climb^2;
LD_climb = CL_climb/CD_climb;

% Cruise
V_cruise_design_ms = Vcruise_objective_ms;

q_cruise = 0.5*rho_cruise*V_cruise_design_ms^2;

CL_cruise = beta_cruise*WS_design_Nm2/q_cruise;
CD_cruise = CD0_clean_sel + K_clean*CL_cruise^2;
LD_cruise = CL_cruise/CD_cruise;

% Landing
Vs_L_ms = stallSpeedSI( ...
    WS_design_Nm2,beta_landing,rho_field,CLmax_L);

V_approach_ms = ks_landing*Vs_L_ms;

CL_landing = CLmax_L/ks_landing^2;
CD_landing = CD0_L_GD_sel + K_L*CL_landing^2;
LD_landing = CL_landing/CD_landing;

%% ========================================================================
% 10. PERFORMANCE - OVERALL OUTPUTS
% ========================================================================

% 10A. Takeoff and landing distances
CD_ground_TO_sel = CD0_TO_GD_sel + K_TO*CL_ground_TO^2;

[sTO_SL_hot_m,sG_TO_SL_m,sTr_TO_SL_m,sCl_TO_SL_m,hTr_TO_SL_m] = ...
    physicsTakeoffDistance_SI( ...
    WS_design_Nm2,PW_design_WN,h_obstacle_m,rho_field,CLmax_TO, ...
    CL_ground_TO,CD_ground_TO_sel,mu_roll,eta_TO,powerFrac_TO, ...
    k_rotation,k_transition,n_transition,gamma_takeoff_deg);

[sTO_5000_hot_m,sG_TO_5000_m,sTr_TO_5000_m,sCl_TO_5000_m,hTr_TO_5000_m] = ...
    physicsTakeoffDistance_SI( ...
    WS_design_Nm2,PW_design_WN,h_obstacle_m,rho_5000,CLmax_TO, ...
    CL_ground_TO,CD_ground_TO_sel,mu_roll,eta_TO,powerFrac_TO, ...
    k_rotation,k_transition,n_transition,gamma_takeoff_deg);

[sL_SL_hot_m,sG_L_SL_m,sFl_L_SL_m,sApp_L_SL_m,hFl_L_SL_m] = ...
    landingDistanceExtremeSTOL_SI( ...
    WS_design_Nm2,beta_landing,rho_field,CLmax_L, ...
    gamma_approach_deg,k_touchdown,stop_decel_g,h_obstacle_m,n_flare);

[sL_5000_hot_m,sG_L_5000_m,sFl_L_5000_m,sApp_L_5000_m,hFl_L_5000_m] = ...
    landingDistanceExtremeSTOL_SI( ...
    WS_design_Nm2,beta_landing,rho_5000,CLmax_L, ...
    gamma_approach_deg,k_touchdown,stop_decel_g,h_obstacle_m,n_flare);

% 10B. Best AEO rate of climb
[ROC_best_ms,V_bestROC_ms] = bestROC_SI( ...
    WS_design_Nm2,PW_design_WN,beta_climb,rho_field,eta_climb, ...
    CD0_TO_GU_sel,e_TO,AR,CLmax_TO,powerFrac_climb,1.0);

% 10C. CLoT best rate of climb
rho_400ft = atmDensitySI(400*0.3048,dT_field_C);

[ROC_CLoT_ms,V_bestCLoT_ms] = bestROC_SI( ...
    WS_design_Nm2,PW_design_WN,beta_climb,rho_400ft,eta_climb, ...
    CD0_TO_GU_sel+dCD0_CLoT,e_CLoT,AR,CLmax_TO, ...
    powerFrac_climb,CLoT_power_remaining);

% 10D. Battery-only engine-out range at 5000 ft
eta_electric_total = ...
    eta_battery*eta_wiring*eta_controller*eta_motor*eta_prop_elec;

[R_engineout_km,V_engineout_best_ms,LDmax_engineout] = ...
    electricEngineOutRange_SI( ...
    W_TO_N,WS_design_Nm2,rho_5000,CD0_clean_sel,e_clean,AR, ...
    m_battery_kg,battery_specific_energy_Whkg, ...
    battery_usable_fraction,eta_electric_total);

% 10E. Maximum-bank maneuver check at selected design
if use_maneuver_constraint
    PW_maneuver_selected_WN = interp1( ...
        WS_Nm2,PW_maneuver_WN,WS_design_Nm2,'linear');
    maneuverPassSelected = ...
        isfinite(PW_maneuver_selected_WN) && ...
        PW_design_WN >= PW_maneuver_selected_WN;
    phi_sustained_available_deg = maxSustainedBank_SI( ...
        WS_design_Nm2,PW_design_WN,beta_maneuver,rho_maneuver, ...
        eta_maneuver,CD0_clean_sel,e_clean,AR,CLmax_clean, ...
        V_maneuver_ms,powerFrac_maneuver);
else
    PW_maneuver_selected_WN = NaN;
    maneuverPassSelected = true;
    phi_sustained_available_deg = NaN;
end

% 10F. Service-ceiling check at selected design
if use_ceiling_constraint
    PW_ceiling_selected_WN = interp1( ...
        WS_Nm2,PW_ceiling_WN,WS_design_Nm2,'linear');
    V_ceiling_constraint_selected_ms = interp1( ...
        WS_Nm2,V_ceiling_best_grid_ms,WS_design_Nm2,'linear');
    [ROC_ceiling_available_ms,V_bestROC_ceiling_ms] = bestROC_SI( ...
        WS_design_Nm2,PW_design_WN,beta_ceiling,rho_ceiling, ...
        eta_ceiling,CD0_clean_sel,e_clean,AR,CLmax_clean, ...
        powerFrac_ceiling,1.0);
    ceilingPassSelected = ROC_ceiling_available_ms >= ROC_ceiling_ms;
else
    PW_ceiling_selected_WN = NaN;
    V_ceiling_constraint_selected_ms = NaN;
    ROC_ceiling_available_ms = NaN;
    V_bestROC_ceiling_ms = NaN;
    ceilingPassSelected = true;
end

% 10G. Correction factors
corr_takeoff_CLoT = beta_climb / ...
    (CLoT_power_remaining*powerFrac_climb);

corr_balked = beta_landing/powerFrac_balked;

%% ========================================================================
% 11. PRINT PERFORMANCE - OVERALL
% ========================================================================

fprintf('\n============================================================\n');
fprintf('AE481 ASSIGNMENT 4 - SI PRELIMINARY DESIGN\n');
fprintf('============================================================\n');

fprintf('Governing constraint : %s\n',governingConstraint);
fprintf('Takeoff model        : 3-phase physics (ground + transition + obstacle climb)\n');
fprintf('Takeoff gamma / n    : %.1f deg / %.2f\n',gamma_takeoff_deg,n_transition);
fprintf('Landing model        : 3-phase physics (approach + flare + ground roll)\n');
fprintf('Landing gamma / n    : %.1f deg / %.2f\n',gamma_approach_deg,n_flare);

fprintf('W/S                  : %.2f N/m^2\n',WS_design_Nm2);
fprintf('W/S (workbook form)  : %.2f kg/m^2\n',massLoading_design_kgm2);

fprintf('P/W                  : %.4f W/N\n',PW_design_WN);
fprintf('P/m                  : %.4f kW/kg\n',specificPower_design_kWkg);
fprintf('W/P (workbook form)  : %.4f kg/kW\n',WP_design_kgkW);

fprintf('Wing reference area  : %.2f m^2\n',S_design_m2);
fprintf('Installed shaft power: %.1f kW\n',P_installed_kW);

fprintf('Clean CD0            : %.5f\n',CD0_clean_sel);
fprintf('Estimated Swet       : %.2f m^2\n',Swet_m2);

fprintf('\n============================================================\n');
fprintf('PERFORMANCE - OVERALL : VALUES TO ENTER\n');
fprintf('============================================================\n');

fprintf('Design range          : %.2f km\n',range_design_km);
fprintf('Electric range target : %.2f km (210 nmi at 225 kt)\n',range_electric_km);
fprintf('Electric range modeled: %.2f km at 225 kt\n',R_electric_selected_km);
fprintf('Electric range margin : %+.2f km\n',electricRangeMargin_km);
fprintf('Electric range pass   : %d\n',electricRangePassSelected);
fprintf('Combined overlap exists: %d\n',combinedFeasibleExists);
fprintf('Engines-out range     : %.2f km\n',R_engineout_km);

fprintf('Cruise altitude       : %.1f m\n',h_cruise_m);
fprintf('Cruise speed          : %.2f m/s\n',V_cruise_design_ms);

fprintf('Maximum bank target   : %.1f deg (n = %.3f)\n', ...
    phi_maneuver_deg,n_maneuver);
fprintf('Maneuver check point  : %.1f m / %.1f m/s\n', ...
    h_maneuver_m,V_maneuver_ms);
fprintf('Maneuver required P/W : %.3f W/N\n',PW_maneuver_selected_WN);
fprintf('Power/lift bank capability: %.1f deg (limit/target %.1f deg)\n', ...
    phi_sustained_available_deg,phi_maneuver_deg);
fprintf('Maneuver pass         : %d\n',maneuverPassSelected);

fprintf('Service ceiling target: %.0f m (%.0f ft)\n', ...
    h_ceiling_m,h_ceiling_m/0.3048);
fprintf('Ceiling ROC criterion : %.3f m/s (%.0f ft/min)\n', ...
    ROC_ceiling_ms,ROC_ceiling_ms/0.00508);
fprintf('Ceiling required P/W  : %.3f W/N @ %.1f m/s\n', ...
    PW_ceiling_selected_WN,V_ceiling_constraint_selected_ms);
fprintf('Ceiling ROC available : %.3f m/s @ %.1f m/s\n', ...
    ROC_ceiling_available_ms,V_bestROC_ceiling_ms);
fprintf('Ceiling pass          : %d\n',ceilingPassSelected);

fprintf('Takeoff W/P           : %.4f kg/kW\n',WP_design_kgkW);
fprintf('Takeoff W/S           : %.2f kg/m^2\n',massLoading_design_kgm2);

fprintf('Takeoff CLoT correction : %.4f\n',corr_takeoff_CLoT);
fprintf('Takeoff climb CD0       : %.5f\n', ...
    CD0_TO_GU_sel+dCD0_CLoT);
fprintf('Takeoff climb CLmax     : %.3f\n',CLmax_TO);

fprintf('Landing correction      : %.4f\n',corr_balked);
fprintf('Landing CD0              : %.5f\n',CD0_L_GD_sel);
fprintf('Landing CLmax            : %.3f\n',CLmax_L);

fprintf('Best rate of climb       : %.3f m/s @ %.2f m/s\n', ...
    ROC_best_ms,V_bestROC_ms);

fprintf('CLoT rate of climb       : %.3f m/s @ %.2f m/s\n', ...
    ROC_CLoT_ms,V_bestCLoT_ms);

fprintf('Takeoff distance         : %.2f m  [physics model]\n',sTO_SL_hot_m);
fprintf('  Ground roll            : %.2f m\n',sG_TO_SL_m);
fprintf('  Transition             : %.2f m\n',sTr_TO_SL_m);
fprintf('  Obstacle climb         : %.2f m\n',sCl_TO_SL_m);
fprintf('  Transition height      : %.2f m\n',hTr_TO_SL_m);

fprintf('Landing distance         : %.2f m  [physics model]\n',sL_SL_hot_m);
fprintf('  Approach               : %.2f m\n',sApp_L_SL_m);
fprintf('  Flare                  : %.2f m\n',sFl_L_SL_m);
fprintf('  Ground roll            : %.2f m\n',sG_L_SL_m);
fprintf('  Flare height           : %.2f m\n',hFl_L_SL_m);

fprintf('\n5000-ft / ISA+18F reporting condition:\n');
fprintf('Takeoff distance         : %.2f m  [physics model]\n',sTO_5000_hot_m);
fprintf('  Ground roll            : %.2f m\n',sG_TO_5000_m);
fprintf('  Transition             : %.2f m\n',sTr_TO_5000_m);
fprintf('  Obstacle climb         : %.2f m\n',sCl_TO_5000_m);
fprintf('Landing distance         : %.2f m  [physics model]\n',sL_5000_hot_m);
fprintf('  Approach               : %.2f m\n',sApp_L_5000_m);
fprintf('  Flare                  : %.2f m\n',sFl_L_5000_m);
fprintf('  Ground roll            : %.2f m\n',sG_L_5000_m);

%% ========================================================================
% 12. PRINT PERFORMANCE - AERODYNAMICS
% ========================================================================

fprintf('\n============================================================\n');
fprintf('PERFORMANCE - AERODYNAMICS : SI VALUES\n');
fprintf('============================================================\n');

fprintf('%-18s %12s %12s %12s %12s\n', ...
    'Parameter','Takeoff','Climb','Cruise','Landing');

fprintf('%-18s %12.4f %12.4f %12.4f %12.4f\n', ...
    'CL max',CLmax_TO,CLmax_TO,CLmax_clean,CLmax_L);

fprintf('%-18s %12.4f %12.4f %12.4f %12.4f\n', ...
    'e',e_TO,e_TO,e_clean,e_L);

fprintf('%-18s %12.5f %12.5f %12.5f %12.5f\n', ...
    'CD0',CD0_TO_GD_sel,CD0_TO_GU_sel,CD0_clean_sel,CD0_L_GD_sel);

fprintf('%-18s %12.3f %12.3f %12.3f %12.3f\n', ...
    'L/D',LD_TO,LD_climb,LD_cruise,LD_landing);

fprintf('%-18s %12.2f %12.2f %12.2f %12.2f\n', ...
    'Airspeed [m/s]',V_TO_ms,V_climb_ms,V_cruise_design_ms,V_approach_ms);

%% ========================================================================
% 13. EXPORT SI RESULT TABLES
% ========================================================================

Performance_Overall_SI = table( ...
    range_design_km, ...
    range_electric_km, ...
    R_electric_selected_km, ...
    R_engineout_km, ...
    h_cruise_m, ...
    V_cruise_design_ms, ...
    phi_maneuver_deg, ...
    phi_sustained_available_deg, ...
    maneuverPassSelected, ...
    h_ceiling_m, ...
    ROC_ceiling_ms, ...
    ROC_ceiling_available_ms, ...
    ceilingPassSelected, ...
    WP_design_kgkW, ...
    massLoading_design_kgm2, ...
    corr_takeoff_CLoT, ...
    CD0_TO_GU_sel+dCD0_CLoT, ...
    CLmax_TO, ...
    corr_balked, ...
    CD0_L_GD_sel, ...
    CLmax_L, ...
    ROC_best_ms, ...
    ROC_CLoT_ms, ...
    sTO_SL_hot_m, ...
    sG_TO_SL_m, ...
    sTr_TO_SL_m, ...
    sCl_TO_SL_m, ...
    hTr_TO_SL_m, ...
    sL_SL_hot_m, ...
    sG_L_SL_m, ...
    sFl_L_SL_m, ...
    sApp_L_SL_m, ...
    hFl_L_SL_m, ...
    'VariableNames',{ ...
    'DesignRange_km', ...
    'ElectricRangeTarget_km', ...
    'ElectricRangeModeled225kt_km', ...
    'EngineOutRange_km', ...
    'CruiseAltitude_m', ...
    'CruiseSpeed_ms', ...
    'MaxBankTarget_deg', ...
    'PowerLiftBankCapability_deg', ...
    'ManeuverPass', ...
    'ServiceCeiling_m', ...
    'CeilingROCRequired_ms', ...
    'CeilingROCAvailable_ms', ...
    'CeilingPass', ...
    'Takeoff_WP_kgkW', ...
    'Takeoff_WS_kgm2', ...
    'TakeoffClimbCorrection', ...
    'TakeoffClimb_CD0', ...
    'TakeoffClimb_CLmax', ...
    'LandingCorrection', ...
    'Landing_CD0', ...
    'Landing_CLmax', ...
    'BestROC_ms', ...
    'CLoT_ROC_ms', ...
    'TakeoffDistance_m', ...
    'TakeoffGroundRoll_m', ...
    'TakeoffTransition_m', ...
    'TakeoffObstacleClimb_m', ...
    'TakeoffTransitionHeight_m', ...
    'LandingDistance_m', ...
    'LandingGroundRoll_m', ...
    'LandingFlare_m', ...
    'LandingApproach_m', ...
    'LandingFlareHeight_m'});

writetable( ...
    Performance_Overall_SI, ...
    'Performance_Overall_Results_SI.csv');

Configuration = ["Takeoff";"Climb";"Cruise";"Landing"];

CLmax_out = [CLmax_TO;CLmax_TO;CLmax_clean;CLmax_L];
e_out     = [e_TO;e_TO;e_clean;e_L];

CD0_out = [ ...
    CD0_TO_GD_sel;
    CD0_TO_GU_sel;
    CD0_clean_sel;
    CD0_L_GD_sel];

LD_out = [LD_TO;LD_climb;LD_cruise;LD_landing];

V_out_ms = [ ...
    V_TO_ms;
    V_climb_ms;
    V_cruise_design_ms;
    V_approach_ms];

Performance_Aerodynamics_SI = table( ...
    Configuration, ...
    CLmax_out, ...
    e_out, ...
    CD0_out, ...
    LD_out, ...
    V_out_ms, ...
    'VariableNames',{ ...
    'Configuration','CLmax','e','CD0','L_D','Airspeed_ms'});

writetable( ...
    Performance_Aerodynamics_SI, ...
    'Performance_Aerodynamics_Results_SI.csv');

%% ========================================================================
% 14. REQUIRED SI P/W - W/S CONSTRAINT DIAGRAM
% ========================================================================

finiteEnvelope = PW_envelope_WN(combinedFeasible & isfinite(PW_envelope_WN));
if isempty(finiteEnvelope)
    finiteEnvelope = PW_envelope_WN(isfinite(PW_envelope_WN));
end

PWplotMax = max([ ...
    40, ...
    1.20*PW_design_WN, ...
    1.10*max(finiteEnvelope)]);

figure('Color','w','Position',[80 80 1200 760]);

hold on;
grid on;
box on;

if any(combinedFeasible)
    idx = find(combinedFeasible);
    fill( ...
        [WS_Nm2(idx),fliplr(WS_Nm2(idx))], ...
        [max(PW_envelope_WN(idx),PW_cruise_250_WN(idx)),PWplotMax*ones(size(idx))], ...
        [0.88 0.94 0.90], ...
        'FaceAlpha',0.55, ...
        'EdgeColor','none', ...
        'DisplayName','Feasible: thresholds + electric range');
else
    text(0.03,0.94, ...
        'No overlap with 210 nmi electric range at 225 kt', ...
        'Units','normalized', ...
        'FontWeight','bold', ...
        'BackgroundColor','w');
end

PW_takeoff_plot_WN = PW_takeoff_WN;
PW_takeoff_plot_WN(~takeoffGeometryOK) = NaN;
plot(WS_Nm2,PW_takeoff_plot_WN,'LineWidth',1.8, ...
    'DisplayName','Physics takeoff <= 91.44 m');

plot(WS_Nm2,PW_RFP_ROC_WN,'LineWidth',1.8, ...
    'DisplayName','RFP initial climb >= 7.62 m/s');

plot(WS_Nm2,PW_initial_WN,'LineWidth',1.8, ...
    'DisplayName','FAA/EASA initial climb >= 4%');

plot(WS_Nm2,PW_CLoT_WN,'LineWidth',1.8, ...
    'DisplayName',sprintf('FAA/EASA CLoT climb >= %.0f%%',100*G_CLoT));

plot(WS_Nm2,PW_balked_WN,'LineWidth',1.8, ...
    'DisplayName','FAA/EASA balked landing >= 3%');

plot(WS_Nm2,PW_cruise_225_WN,'LineWidth',1.8, ...
    'DisplayName','RFP cruise >= 115.75 m/s');

plot(WS_Nm2,PW_cruise_250_WN,'--','LineWidth',1.6, ...
    'DisplayName','RFP objective cruise >= 128.61 m/s');

if use_maneuver_constraint
    PW_maneuver_plot_WN = PW_maneuver_WN;
    PW_maneuver_plot_WN(~maneuverLiftOK) = NaN;
    plot(WS_Nm2,PW_maneuver_plot_WN,'LineWidth',1.8, ...
        'DisplayName',sprintf('Sustained %.0f deg bank @ %.0f kt', ...
        phi_maneuver_deg,V_maneuver_ms/0.514444));
end

if use_ceiling_constraint
    plot(WS_Nm2,PW_ceiling_WN,'LineWidth',1.8, ...
        'DisplayName',sprintf('Service ceiling %.0f kft / %.0f fpm', ...
        h_ceiling_m/0.3048/1000,ROC_ceiling_ms/0.00508));
end

plot( ...
    [WS_landing_max_Nm2 WS_landing_max_Nm2], ...
    [0 PWplotMax], ...
    '--','LineWidth',1.8, ...
    'DisplayName','RFP landing <= 91.44 m');

if use_stall_constraint
    plot( ...
        [WS_stall_max_Nm2 WS_stall_max_Nm2], ...
        [0 PWplotMax], ...
        ':','LineWidth',1.8, ...
        'DisplayName',sprintf('Selected stall <= %.1f m/s', ...
        Vstall_limit_ms));
end

if combinedFeasibleExists && electricRangePassSelected
    plot( ...
        WS_design_Nm2,PW_design_WN,'kp', ...
        'MarkerSize',14, ...
        'MarkerFaceColor','y', ...
        'DisplayName','Selected design');
else
    plot( ...
        WS_design_Nm2,PW_design_WN,'rp', ...
        'MarkerSize',14, ...
        'MarkerFaceColor','r', ...
        'DisplayName','Diagnostic point - electric range not met');
end

text( ...
    WS_design_Nm2+40,PW_design_WN, ...
    sprintf(' W/S = %.0f N/m^2\\n P/W = %.2f W/N', ...
    WS_design_Nm2,PW_design_WN), ...
    'FontWeight','bold');

xlabel('Takeoff Wing Loading, W/S [N/m^2]');
ylabel('Installed Power-to-Weight, P/W [W/N]');

title('M-BRAER Preliminary P/W - W/S Constraint Diagram (SI)');

xlim([min(WS_Nm2), ...
      min(max(WS_Nm2),1.20*WS_vertical_limit_Nm2)]);

ylim([0,PWplotMax]);

legend('Location','eastoutside');

exportFigure('AE481_Assignment4_PW_WS_SI');

%% ========================================================================
% 14B. ELECTRIC RANGE AT 225 kt VERSUS W/S
% ========================================================================

figure('Color','w','Position',[100 100 1000 700]);
hold on;
grid on;
box on;

plot(WS_Nm2,R_electric_grid_m/1000,'LineWidth',2, ...
    'DisplayName','Modeled electric range at 225 kt');

yline(range_electric_km,'--','210 nmi requirement', ...
    'LineWidth',1.6,'DisplayName','Electric range requirement');

xline(WS_landing_max_Nm2,'--','300-ft landing W/S limit', ...
    'LineWidth',1.6,'DisplayName','Landing W/S limit');

plot(WS_design_Nm2,R_electric_selected_km,'kp', ...
    'MarkerSize',12,'MarkerFaceColor','y', ...
    'DisplayName','Selected / diagnostic point');

xlabel('Takeoff Wing Loading, W/S [N/m^2]');
ylabel('Electric Range at 225 kt [km]');
title('M-BRAER Electric Cruise Range Screen');
legend('Location','best');

exportFigure('AE481_Assignment4_ElectricRange_SI');

%% ========================================================================
% 15. DRAG-POLAR PLOT
% ========================================================================

figure('Color','w','Position',[100 100 1000 700]);

hold on;
grid on;
box on;

CL = linspace(0,CLmax_clean,250);

plot( ...
    CD0_clean_sel + K_clean*CL.^2, ...
    CL, ...
    'LineWidth',1.8, ...
    'DisplayName','Clean / cruise');

CL = linspace(0,CLmax_TO,300);

plot( ...
    CD0_TO_GU_sel + K_TO*CL.^2, ...
    CL, ...
    'LineWidth',1.8, ...
    'DisplayName','Takeoff flaps, gear up');

plot( ...
    CD0_TO_GD_sel + K_TO*CL.^2, ...
    CL, ...
    '--','LineWidth',1.8, ...
    'DisplayName','Takeoff flaps, gear down');

CL = linspace(0,CLmax_L,350);

plot( ...
    CD0_L_GD_sel + K_L*CL.^2, ...
    CL, ...
    'LineWidth',1.8, ...
    'DisplayName','Landing flaps, gear down');

xlabel('C_D');
ylabel('C_L');

title('M-BRAER Preliminary Drag Polars');

legend('Location','best');

exportFigure('AE481_Assignment4_DragPolars_SI');

fprintf('\nFiles written:\n');
fprintf('  Performance_Overall_Results_SI.csv\n');
fprintf('  Performance_Aerodynamics_Results_SI.csv\n');
fprintf('  AE481_Assignment4_PW_WS_SI.pdf / .png\n');
fprintf('  AE481_Assignment4_ElectricRange_SI.pdf / .png\n');
fprintf('  AE481_Assignment4_DragPolars_SI.pdf / .png\n');
fprintf('============================================================\n');

%% ========================================================================
% LOCAL FUNCTIONS - ALL SI
% ========================================================================

function rho = atmDensitySI(h_m,dT_C)
    T0 = 288.15;
    p0 = 101325;
    L  = 0.0065;
    g0 = 9.80665;
    R  = 287.05287;

    Tisa = T0 - L*h_m;

    p = p0*(Tisa/T0)^(g0/(R*L));

    Tactual = Tisa + dT_C;

    rho = p/(R*Tactual);
end

function Vstall_ms = stallSpeedSI(WS_TO_Nm2,betaW,rho,CLmax)
    WS_local = betaW*WS_TO_Nm2;

    Vstall_ms = sqrt(2*WS_local/(rho*CLmax));
end

function WSmax_TO_Nm2 = stallWsLimitSI( ...
    Vstall_ms,rho,CLmax,betaW)
    WS_condition = 0.5*rho*Vstall_ms^2*CLmax;

    WSmax_TO_Nm2 = WS_condition/betaW;
end

function [PWinstalled_WN,geometryOK,sg_m,str_m,scl_m,htr_m] = ...
    physicsTakeoffPW_SI( ...
    WS_Nm2,sTotalReq_m,hObstacle_m,rho,CLmax,CL_TO,CD_TO,mu,etaProp, ...
    powerAvailFrac,kR,kTr,nTransition,gamma_deg)

    g0 = 9.80665;
    gamma = deg2rad(gamma_deg);

    Vs_ms  = sqrt(2.*WS_Nm2./(rho*CLmax));
    Vtr_ms = kTr.*Vs_ms;

    R_m = Vtr_ms.^2./(g0*(nTransition - 1));
    str_full_m = R_m.*sin(gamma);
    htr_full_m = R_m.*(1 - cos(gamma));

    belowObstacle = htr_full_m < hObstacle_m;

    str_m = zeros(size(WS_Nm2));
    htr_m = zeros(size(WS_Nm2));
    scl_m = zeros(size(WS_Nm2));

    str_m(belowObstacle) = str_full_m(belowObstacle);
    htr_m(belowObstacle) = htr_full_m(belowObstacle);
    scl_m(belowObstacle) = ...
        (hObstacle_m - htr_full_m(belowObstacle))./tan(gamma);

    reachesObstacleInTransition = ~belowObstacle;
    if any(reachesObstacleInTransition)
        Rlocal = R_m(reachesObstacleInTransition);
        thetaObs = acos(1 - hObstacle_m./Rlocal);
        str_m(reachesObstacleInTransition) = Rlocal.*sin(thetaObs);
        htr_m(reachesObstacleInTransition) = hObstacle_m;
        scl_m(reachesObstacleInTransition) = 0;
    end

    sg_m = sTotalReq_m - str_m - scl_m;
    geometryOK = sg_m > 0;

    aeroGroundTerm = ...
        (kR^2/(2*CLmax)).*(CD_TO - mu*CL_TO);

    TW_required = ...
        (kR^2.*WS_Nm2)./(g0*rho*CLmax.*sg_m) + ...
        mu + aeroGroundTerm;

    PWavailable_WN = TW_required.*Vtr_ms./etaProp;
    PWinstalled_WN = PWavailable_WN./powerAvailFrac;

    PWinstalled_WN(~geometryOK) = inf;
end

function [sTotal_m,sg_m,str_m,scl_m,htr_m] = ...
    physicsTakeoffDistance_SI( ...
    WS_Nm2,PWinstalled_WN,hObstacle_m,rho,CLmax,CL_TO,CD_TO,mu,etaProp, ...
    powerAvailFrac,kR,kTr,nTransition,gamma_deg)

    g0 = 9.80665;
    gamma = deg2rad(gamma_deg);

    Vs_ms  = sqrt(2.*WS_Nm2./(rho*CLmax));
    Vtr_ms = kTr.*Vs_ms;

    PWavailable_WN = PWinstalled_WN.*powerAvailFrac;
    TW_available = etaProp.*PWavailable_WN./Vtr_ms;

    aeroGroundTerm = ...
        (kR^2/(2*CLmax)).*(CD_TO - mu*CL_TO);

    accelBracket = TW_available - mu - aeroGroundTerm;

    if any(accelBracket <= 0)
        sg_m = inf(size(WS_Nm2));
    else
        sg_m = ...
            (kR^2.*WS_Nm2)./(g0*rho*CLmax.*accelBracket);
    end

    R_m = Vtr_ms.^2./(g0*(nTransition - 1));
    str_full_m = R_m.*sin(gamma);
    htr_full_m = R_m.*(1 - cos(gamma));

    if htr_full_m < hObstacle_m
        str_m = str_full_m;
        htr_m = htr_full_m;
        scl_m = (hObstacle_m - htr_m)./tan(gamma);
    else
        thetaObs = acos(1 - hObstacle_m./R_m);
        str_m = R_m.*sin(thetaObs);
        htr_m = hObstacle_m;
        scl_m = 0;
    end

    sTotal_m = sg_m + str_m + scl_m;
end

function [sTotal_m, sg_m, sfl_m, sapp_m, hfl_m] = ...
    landingDistanceExtremeSTOL_SI( ...
    WS_TO_Nm2, betaW, rho, CLmax, gamma_deg, kTD, decel_g, hObstacle_m, nFlare, safetyFactor)
% Physics-based 3-phase extreme-STOL landing model:
%   1) steady approach at gamma_deg from obstacle height down to flare height,
%   2) circular flare transition at V_TD = kTD * V_s with load factor nFlare,
%   3) deceleration ground roll under constant braking (decel_g).

    if nargin < 9 || isempty(nFlare)
        nFlare = 1.15; % Default flare load factor
    end
    if nargin < 10 || isempty(safetyFactor)
        safetyFactor = 1.0; % Default to raw physical distance
    end

    g0 = 9.80665;
    gamma = deg2rad(gamma_deg);

    WS_L_Nm2 = betaW .* WS_TO_Nm2;
    Vs_ms = sqrt(2 .* WS_L_Nm2 ./ (rho * CLmax));
    VTD_ms = kTD .* Vs_ms;

    % Flare transition radius from centripetal acceleration
    R_m = VTD_ms.^2 ./ (g0 * (nFlare - 1));
    sfl_full_m = R_m .* sin(gamma);
    hfl_full_m = R_m .* (1 - cos(gamma));

    belowObstacle = hfl_full_m < hObstacle_m;

    sfl_m  = zeros(size(WS_TO_Nm2));
    hfl_m  = zeros(size(WS_TO_Nm2));
    sapp_m = zeros(size(WS_TO_Nm2));

    sfl_m(belowObstacle)  = sfl_full_m(belowObstacle);
    hfl_m(belowObstacle)  = hfl_full_m(belowObstacle);
    sapp_m(belowObstacle) = (hObstacle_m - hfl_full_m(belowObstacle)) ./ tan(gamma);

    reachesObstacleInFlare = ~belowObstacle;
    if any(reachesObstacleInFlare)
        Rlocal = R_m(reachesObstacleInFlare);
        thetaObs = acos(1 - hObstacle_m ./ Rlocal);
        sfl_m(reachesObstacleInFlare)  = Rlocal .* sin(thetaObs);
        hfl_m(reachesObstacleInFlare)  = hObstacle_m;
        sapp_m(reachesObstacleInFlare) = 0;
    end

    % Ground roll deceleration
    aStop_ms2 = decel_g * g0;
    sg_m = VTD_ms.^2 ./ (2 * aStop_ms2);

    sTotal_m = (sg_m + sfl_m + sapp_m) .* safetyFactor;
end

function PW_TO_WN = climbPWfromGradient_SI( ...
    WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,CLmax,ks,G, ...
    powerRemaining,powerAvailFrac)

    WS_local_Nm2 = betaW.*WS_TO_Nm2;

    CL = CLmax/ks^2;

    V_ms = sqrt(2.*WS_local_Nm2./(rho*CL));

    K = 1/(pi*AR*e);

    CD = CD0 + K*CL^2;

    D_over_W = CD./CL;

    PW_current_WN = ...
        V_ms./eta .* (D_over_W + G);

    PW_TO_WN = ...
        betaW.*PW_current_WN ./ ...
        (powerRemaining*powerAvailFrac);
end

function PW_TO_WN = climbPWfromROC_SI( ...
    WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,CLmax,ks,ROC_ms, ...
    powerRemaining,powerAvailFrac)

    WS_local_Nm2 = betaW.*WS_TO_Nm2;

    CL = CLmax/ks^2;

    V_ms = sqrt(2.*WS_local_Nm2./(rho*CL));

    G = ROC_ms./V_ms;

    PW_TO_WN = climbPWfromGradient_SI( ...
        WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,CLmax,ks,G, ...
        powerRemaining,powerAvailFrac);
end

function PW_TO_WN = cruisePW_SI( ...
    WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,V_ms,powerAvailFrac)
% Metabook Sec. 4.10.2, Eq. (4.41), expressed in SI units.
% WS_TO_Nm2 = W_takeoff/S [N/m^2]; output = P_installed/W_takeoff [W/N].
% betaW = W_cruise/W_takeoff; powerAvailFrac = P_cruise/P_installed.
% CD0 is the scalar reference cruise parasite-drag coefficient.
% No factor of 550: that conversion applies only to hp and FPS units.

    q_Pa = 0.5*rho*V_ms^2;
    K = 1/(pi*AR*e);

    parasiteTerm = q_Pa*CD0./WS_TO_Nm2;
    inducedTerm = K*betaW^2.*WS_TO_Nm2./q_Pa;

    PW_TO_WN = V_ms/(eta*powerAvailFrac) .* ...
        (parasiteTerm + inducedTerm);
end

function [PW_TO_WN,liftOK] = maneuverPW_SI( ...
    WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,CLmax,V_ms,n,powerAvailFrac)

    q_Pa = 0.5*rho*V_ms^2;
    K = 1/(pi*AR*e);

    CL = n.*betaW.*WS_TO_Nm2./q_Pa;
    liftOK = CL <= CLmax;

    CD = CD0 + K.*CL.^2;

    PW_TO_WN = ...
        q_Pa*V_ms.*CD ./ ...
        (eta.*WS_TO_Nm2.*powerAvailFrac);

    PW_TO_WN(~liftOK) = inf;
end

function [PW_TO_WN,Vbest_ms,CLbest] = serviceCeilingPW_SI( ...
    WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,CLmax,ksMin,ROC_req_ms, ...
    powerAvailFrac)

    K = 1/(pi*AR*e);

    CL_minPower = sqrt(3.*CD0./K);
    CL_stallMargin = CLmax/(ksMin^2);
    CLbest = min(CL_minPower,CL_stallMargin);

    WS_local_Nm2 = betaW.*WS_TO_Nm2;
    Vbest_ms = sqrt(2.*WS_local_Nm2./(rho.*CLbest));

    CD = CD0 + K.*CLbest.^2;
    D_over_W_current = CD./CLbest;

    PW_TO_WN = ...
        betaW.*(Vbest_ms.*D_over_W_current + ROC_req_ms) ./ ...
        (eta.*powerAvailFrac);
end

function phi_deg = maxSustainedBank_SI( ...
    WS_TO_Nm2,PWinstalled_WN,betaW,rho,eta,CD0,e,AR,CLmax, ...
    V_ms,powerAvailFrac)

    q_Pa = 0.5*rho*V_ms^2;
    K = 1/(pi*AR*e);

    CD_power = ...
        eta*PWinstalled_WN*powerAvailFrac*WS_TO_Nm2/(q_Pa*V_ms);

    inducedAllowance = max(CD_power - CD0,0);

    if inducedAllowance <= 0
        phi_deg = 0;
        return;
    end

    n_power = sqrt( ...
        inducedAllowance*q_Pa^2 / ...
        (K*(betaW*WS_TO_Nm2)^2));

    n_lift = q_Pa*CLmax/(betaW*WS_TO_Nm2);
    n_max = min(n_power,n_lift);

    if n_max <= 1
        phi_deg = 0;
    else
        phi_deg = acosd(1/n_max);
    end
end

function [ROCmax_ms,Vbest_ms] = bestROC_SI( ...
    WS_TO_Nm2,PWinstalled_WN,betaW,rho,eta,CD0,e,AR,CLmax, ...
    powerAvailFrac,powerRemaining)

    WS_local_Nm2 = betaW*WS_TO_Nm2;

    Vs_ms = sqrt(2*WS_local_Nm2/(rho*CLmax));

    V_ms = linspace(1.05*Vs_ms,140,2500);

    q_Pa = 0.5*rho.*V_ms.^2;

    CL = WS_local_Nm2./q_Pa;

    K = 1/(pi*AR*e);

    CD = CD0 + K.*CL.^2;

    valid = CL <= CLmax;

    D_over_W = CD./CL;

    PW_current_WN = ...
        PWinstalled_WN*powerAvailFrac*powerRemaining/betaW;

    specificPowerAvailable_ms = eta*PW_current_WN;

    ROC_ms = ...
        specificPowerAvailable_ms - D_over_W.*V_ms;

    ROC_ms(~valid) = -inf;

    [ROCmax_ms,idx] = max(ROC_ms);

    Vbest_ms = V_ms(idx);
end

function [R_km,Vbest_ms,LDmax] = electricEngineOutRange_SI( ...
    W_N,WS_TO_Nm2,rho,CD0,e,AR,mBattery_kg,specificEnergy_Whkg, ...
    usableFraction,etaTotal)

    K = 1/(pi*AR*e);

    CL_LDmax = sqrt(CD0/K);

    LDmax = ...
        CL_LDmax/(CD0 + K*CL_LDmax^2);

    Vbest_ms = ...
        sqrt(2*WS_TO_Nm2/(rho*CL_LDmax));

    E_usable_Wh = ...
        mBattery_kg*specificEnergy_Whkg*usableFraction;

    E_propulsive_J = ...
        E_usable_Wh*3600*etaTotal;

    D_N = W_N/LDmax;

    R_m = E_propulsive_J/D_N;

    R_km = R_m/1000;
end

function exportFigure(baseName)
    try
        exportgraphics( ...
            gcf,[baseName '.pdf'],'ContentType','vector');

        exportgraphics( ...
            gcf,[baseName '.png'],'Resolution',600);
    catch
        set(gcf,'PaperPositionMode','auto');

        print( ...
            gcf,[baseName '.pdf'],'-dpdf','-painters');

        print( ...
            gcf,[baseName '.png'],'-dpng','-r600');
    end
end