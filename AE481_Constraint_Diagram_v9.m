%% AE481 Assignment 4 - Parametric SI Sizing
% M-BRAER Hybrid-Electric STOL Commuter
%
% SI-UNIT VERSION & WORKSPACE INTEGRATED
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
% No toolbox is required.

%% ========================================================================
% 1. LOAD INPUTS AND WEIGHTS FROM WORKSPACE OR RUN DEPENDENCIES
% ========================================================================

g0 = 9.80665;                            % [m/s^2]

if ~exist('in', 'var')
    fprintf('Running aircraft_inputs to populate design parameters...\n');
    aircraft_inputs;
end

if ~exist('values_kg', 'var')
    fprintf('Running Weight_Predict to generate weight vector...\n');
    values_kg = Weight_Predict(in);
end

m_TO_kg              = values_kg(1);
m_empty_kg           = values_kg(2);
m_fuel_kg            = values_kg(3);
m_crew_pax_kg        = values_kg(4);
m_baggage_kg         = values_kg(5);
m_electric_system_kg = values_kg(6);
m_battery_kg         = values_kg(7);

W_TO_N = m_TO_kg * g0;                     % takeoff weight force [N]
disp(W_TO_N);

% Current mission values from 'in' structure
range_design_km   = in.range_nmi * 1.852;           
range_electric_km = in.electricRange_nmi * 1.852;   

%% ========================================================================
% 2. RFP REQUIREMENTS - SI UNITS (Unpacked from 'in')
% ========================================================================

Vcruise_threshold_ms = in.Vcruise_threshold_ms;     
Vcruise_objective_ms = in.Vcruise_objective_ms;     

s_TO_req_m   = in.s_TO_req_m;                 
s_L_req_m    = in.s_L_req_m;                 

ROC_RFP_ms   = in.ROC_RFP_ms;               

h_obstacle_m = in.h_obstacle_m;                

% Required field condition: sea level, ISA + 18 F = ISA + 10 C
h_field_m    = in.h_field_m;
dT_field_C   = in.dT_field_C;

% RFP 5000-ft hot-day reporting condition
h_5000_m     = in.h_5000_m;                  
dT_5000_C    = in.dT_5000_C;

%% ========================================================================
% 3. SELECTED DESIGN ASSUMPTIONS (Unpacked from 'in')
% ========================================================================

%% 3A. General configuration
AR           = in.AR;                               

h_cruise_m   = in.h_cruise_m;              
dT_cruise_C  = in.dT_cruise_C;

% Weight fractions
beta_cruise  = in.beta_cruise;                     
beta_landing = in.beta_landing;                     
beta_climb   = in.beta_climb;

%% 3B. Preliminary high-lift / powered-lift selections
CLmax_clean  = in.CLmax_clean;
CLmax_TO     = in.CLmax_TO;
CLmax_L      = in.CLmax_L;

%% 3C. Preliminary parasite-drag method
wet_c_SI     = in.wet_c_SI;
wet_d        = in.wet_d;

Cf_equiv     = in.Cf_equiv;

% Preliminary drag increments
dCD0_TO_flaps = in.dCD0_TO_flaps;    
dCD0_L_flaps  = in.dCD0_L_flaps;
dCD0_gear     = in.dCD0_gear;

% Span efficiencies
e_clean      = in.e_clean;       
e_TO         = in.e_TO;
e_L          = in.e_L;

%% 3D. Propeller efficiencies / available power
eta_TO       = in.eta_TO;
eta_climb    = in.eta_climb;
eta_cruise   = in.eta_cruise;
eta_balked   = in.eta_balked;

powerFrac_TO       = in.powerFrac_TO;
powerFrac_climb    = in.powerFrac_climb;
powerFrac_cruise   = in.powerFrac_cruise;
powerFrac_balked   = in.powerFrac_balked;

powerFrac_maneuver = in.powerFrac_maneuver;                
powerFrac_ceiling  = in.powerFrac_ceiling;                

%% 3E. Critical Loss of Thrust (CLoT)
CLoT_power_remaining = in.CLoT_power_remaining;

dCD0_CLoT    = in.dCD0_CLoT;     
e_CLoT       = in.e_CLoT;   

%% 3F. FAA/EASA Level-3 climb requirements
isHighSpeed  = in.isHighSpeed; 

G_initial    = in.G_initial;
G_CLoT       = in.G_CLoT;
G_balked     = in.G_balked;

% Stall speed safety margins
ks_takeoff   = in.ks_takeoff;
ks_initial   = in.ks_initial;
ks_CLoT      = in.ks_CLoT;
ks_balked    = in.ks_balked;
ks_landing   = in.ks_landing;

%% 3G. Physics-based extreme-STOL takeoff assumptions
mu_roll            = in.mu_roll;               
k_rotation         = in.k_rotation;         
k_transition       = in.k_transition;       
gamma_takeoff_deg  = in.gamma_takeoff_deg;                 
n_transition       = in.n_transition;              

%% 3H. Extreme-STOL landing assumptions
gamma_approach_deg = in.gamma_approach_deg;
k_touchdown        = in.k_touchdown;               
stop_decel_g       = in.stop_decel_g;               
n_flare            = in.n_flare;              

%% 3I. Battery assumptions for battery-only emergency range
battery_specific_energy_Whkg = in.pack_Whkg;
battery_usable_fraction      = in.usableSOC;

eta_battery    = in.etaBattery;
eta_wiring     = in.etaWiring;
eta_controller = in.etaController;
eta_motor      = in.etaMotor;
eta_prop_elec  = in.etaPropElec;

%% 3J. Maneuver / bank-angle design requirement
use_maneuver_constraint = true;
phi_maneuver_deg  = in.phi_maneuver_deg;                   
n_maneuver        = 1/cosd(phi_maneuver_deg);
h_maneuver_m      = h_cruise_m;          
dT_maneuver_C     = dT_cruise_C;
V_maneuver_ms     = Vcruise_threshold_ms;
beta_maneuver     = beta_cruise;
eta_maneuver      = eta_cruise;

%% 3K. Service-ceiling design requirement
use_ceiling_constraint = true;
h_ceiling_m       = 12500*0.3048;         
dT_ceiling_C      = 0;                    
ROC_ceiling_ms    = 100*0.00508;          
beta_ceiling      = beta_cruise;          
eta_ceiling       = eta_cruise;
ks_ceiling        = 1.10;                

%% 3L. Constraint-chart settings
WS_Nm2 = [linspace(0.1,249.9,1000),linspace(250,4800,1500)]; % [N/m^2]

WS_design_fraction = 0.998; 
PW_design_margin   = 1.002;

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

sLanding_grid_m = landingDistanceExtremeSTOL_SI( ...
    WS_Nm2,beta_landing,rho_field,CLmax_L,gamma_approach_deg, ...
    k_touchdown,stop_decel_g,h_obstacle_m,n_flare);

landingOK = sLanding_grid_m <= s_L_req_m;

if ~any(landingOK)
    error(['No point in the selected W/S range satisfies the 300-ft ', ...
           'landing model. Revisit the STOL assumptions.']);
end

WS_landing_max_Nm2 = max(WS_Nm2(landingOK));

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

K_TO_constraint = 1/(pi*AR*e_TO);
CL_ground_TO = CLmax_TO/k_rotation^2;
CD_ground_TO = CD0_TO_GD + K_TO_constraint*CL_ground_TO^2;

[PW_takeoff_WN,takeoffGeometryOK, ...
    sGround_req_grid_m,sTransition_grid_m,sClimb_grid_m, ...
    hTransition_grid_m] = physicsTakeoffPW_SI( ...
    WS_Nm2,s_TO_req_m,h_obstacle_m,rho_field,CLmax_TO, ...
    CL_ground_TO,CD_ground_TO,mu_roll,eta_TO,powerFrac_TO, ...
    k_rotation,k_transition,n_transition,gamma_takeoff_deg);

PW_RFP_ROC_WN = climbPWfromROC_SI( ...
    WS_Nm2,beta_climb,rho_field,eta_climb, ...
    CD0_TO_GD,e_TO,AR,CLmax_TO,ks_initial,ROC_RFP_ms, ...
    1.0,powerFrac_climb);

PW_initial_WN = climbPWfromGradient_SI( ...
    WS_Nm2,beta_climb,rho_field,eta_climb, ...
    CD0_TO_GD,e_TO,AR,CLmax_TO,ks_initial,G_initial, ...
    1.0,powerFrac_climb);

PW_CLoT_WN = climbPWfromGradient_SI( ...
    WS_Nm2,beta_climb,rho_field,eta_climb, ...
    CD0_TO_GU+dCD0_CLoT,e_CLoT,AR,CLmax_TO,ks_CLoT,G_CLoT, ...
    CLoT_power_remaining,powerFrac_climb);

PW_balked_WN = climbPWfromGradient_SI( ...
    WS_Nm2,beta_landing,rho_field,eta_balked, ...
    CD0_L_GD,e_L,AR,CLmax_L,ks_balked,G_balked, ...
    1.0,powerFrac_balked);

WS_cruise_ref_Nm2 = WS_design_fraction*WS_vertical_limit_Nm2;
S_cruise_ref_m2 = W_TO_N/WS_cruise_ref_Nm2;

CD0_cruise_ref = 0.018;

PW_cruise_225_WN = cruisePW_SI( ...
    WS_Nm2,beta_cruise,rho_cruise,eta_cruise, ...
    CD0_cruise_ref,e_clean,AR,Vcruise_threshold_ms,powerFrac_cruise);

PW_cruise_250_WN = cruisePW_SI( ...
    WS_Nm2,beta_cruise,rho_cruise,eta_cruise, ...
    CD0_cruise_ref,e_clean,AR,Vcruise_objective_ms,powerFrac_cruise);

if use_maneuver_constraint
    [PW_maneuver_WN,maneuverLiftOK] = maneuverPW_SI( ...
        WS_Nm2,beta_maneuver,rho_maneuver,eta_maneuver, ...
        CD0_cruise_ref,e_clean,AR,CLmax_clean,V_maneuver_ms, ...
        n_maneuver,powerFrac_maneuver);
else
    PW_maneuver_WN = zeros(size(WS_Nm2));
    maneuverLiftOK = true(size(WS_Nm2));
end

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
        'screen, and selected maneuver-lift requirement.']);
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

Vs_TO_ms = stallSpeedSI(WS_design_Nm2,beta_climb,rho_field,CLmax_TO);
V_TO_ms = ks_takeoff*Vs_TO_ms;
CL_TO_rep = CLmax_TO/ks_takeoff^2;
CD_TO_rep = CD0_TO_GD_sel + K_TO*CL_TO_rep^2;
LD_TO = CL_TO_rep/CD_TO_rep;

V_climb_ms = ks_initial*Vs_TO_ms;
CL_climb = CLmax_TO/ks_initial^2;
CD_climb = CD0_TO_GU_sel + K_TO*CL_climb^2;
LD_climb = CL_climb/CD_climb;

V_cruise_design_ms = Vcruise_objective_ms;
q_cruise = 0.5*rho_cruise*V_cruise_design_ms^2;
CL_cruise = beta_cruise*WS_design_Nm2/q_cruise;
CD_cruise = CD0_clean_sel + K_clean*CL_cruise^2;
LD_cruise = CL_cruise/CD_cruise;

Vs_L_ms = stallSpeedSI(WS_design_Nm2,beta_landing,rho_field,CLmax_L);
V_approach_ms = ks_landing*Vs_L_ms;
CL_landing = CLmax_L/ks_landing^2;
CD_landing = CD0_L_GD_sel + K_L*CL_landing^2;
LD_landing = CL_landing/CD_landing;

%% ========================================================================
% 10. PERFORMANCE - OVERALL OUTPUTS
% ========================================================================

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

[ROC_best_ms,V_bestROC_ms] = bestROC_SI( ...
    WS_design_Nm2,PW_design_WN,beta_climb,rho_field,eta_climb, ...
    CD0_TO_GU_sel,e_TO,AR,CLmax_TO,powerFrac_climb,1.0);

rho_400ft = atmDensitySI(400*0.3048,dT_field_C);

[ROC_CLoT_ms,V_bestCLoT_ms] = bestROC_SI( ...
    WS_design_Nm2,PW_design_WN,beta_climb,rho_400ft,eta_climb, ...
    CD0_TO_GU_sel+dCD0_CLoT,e_CLoT,AR,CLmax_TO, ...
    powerFrac_climb,CLoT_power_remaining);

eta_electric_total = ...
    eta_battery*eta_wiring*eta_controller*eta_motor*eta_prop_elec;

[R_engineout_km,V_engineout_best_ms,LDmax_engineout] = ...
    electricEngineOutRange_SI( ...
    W_TO_N,WS_design_Nm2,rho_5000,CD0_clean_sel,e_clean,AR, ...
    m_battery_kg,battery_specific_energy_Whkg, ...
    battery_usable_fraction,eta_electric_total);

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

corr_takeoff_CLoT = beta_climb / (CLoT_power_remaining*powerFrac_climb);
corr_balked = beta_landing/powerFrac_balked;

%% ========================================================================
% 11. PRINT PERFORMANCE - OVERALL & AERODYNAMICS (Summary)
% ========================================================================

fprintf('\n============================================================\n');
fprintf('AE481 ASSIGNMENT 4 - SI PRELIMINARY DESIGN (WORKSPACE INTEGRATED)\n');
fprintf('============================================================\n');
fprintf('Governing constraint : %s\n',governingConstraint);
fprintf('W/S                  : %.2f N/m^2 (%.2f kg/m^2)\n',WS_design_Nm2,massLoading_design_kgm2);
fprintf('P/W                  : %.4f W/N (%.4f kW/kg)\n',PW_design_WN,specificPower_design_kWkg);
fprintf('Wing reference area  : %.2f m^2\n',S_design_m2);
fprintf('Installed shaft power: %.1f kW\n',P_installed_kW);
fprintf('Electric range mode  : %.2f km (Target: %.2f km)\n',R_electric_selected_km,range_electric_km);

%% ========================================================================
% 12. EXPORT SI RESULT TABLES
% ========================================================================

Performance_Overall_SI = table( ...
    range_design_km, range_electric_km, R_electric_selected_km, R_engineout_km, ...
    h_cruise_m, V_cruise_design_ms, phi_maneuver_deg, phi_sustained_available_deg, ...
    maneuverPassSelected, h_ceiling_m, ROC_ceiling_ms, ROC_ceiling_available_ms, ...
    ceilingPassSelected, WP_design_kgkW, massLoading_design_kgm2, corr_takeoff_CLoT, ...
    CD0_TO_GU_sel+dCD0_CLoT, CLmax_TO, corr_balked, CD0_L_GD_sel, CLmax_L, ...
    ROC_best_ms, ROC_CLoT_ms, sTO_SL_hot_m, sG_TO_SL_m, sTr_TO_SL_m, sCl_TO_SL_m, ...
    hTr_TO_SL_m, sL_SL_hot_m, sG_L_SL_m, sFl_L_SL_m, sApp_L_SL_m, hFl_L_SL_m, ...
    'VariableNames',{'DesignRange_km','ElectricRangeTarget_km','ElectricRangeModeled225kt_km', ...
    'EngineOutRange_km','CruiseAltitude_m','CruiseSpeed_ms','MaxBankTarget_deg', ...
    'PowerLiftBankCapability_deg','ManeuverPass','ServiceCeiling_m','CeilingROCRequired_ms', ...
    'CeilingROCAvailable_ms','CeilingPass','Takeoff_WP_kgkW','Takeoff_WS_kgm2', ...
    'TakeoffClimbCorrection','TakeoffClimb_CD0','TakeoffClimb_CLmax','LandingCorrection', ...
    'Landing_CD0','Landing_CLmax','BestROC_ms','CLoT_ROC_ms','TakeoffDistance_m', ...
    'TakeoffGroundRoll_m','TakeoffTransition_m','TakeoffObstacleClimb_m','TakeoffTransitionHeight_m', ...
    'LandingDistance_m','LandingGroundRoll_m','LandingFlare_m','LandingApproach_m','LandingFlareHeight_m'});

writetable(Performance_Overall_SI, 'Performance_Overall_Results_SI.csv');

%% ========================================================================
% 13. REQUIRED SI P/W - W/S CONSTRAINT DIAGRAM
% ========================================================================

finiteEnvelope = PW_envelope_WN(combinedFeasible & WS_Nm2 >= 250 & isfinite(PW_envelope_WN));
if isempty(finiteEnvelope)
    finiteEnvelope = PW_envelope_WN(WS_Nm2 >= 250 & isfinite(PW_envelope_WN));
end

PWplotMax = max([40, 1.20*PW_design_WN, 1.10*max(finiteEnvelope)]);

figure('Color','w','Position',[80 80 1200 760]);
hold on;
grid on;
box on;

if any(combinedFeasible)
    idx = find(combinedFeasible);
    fill([WS_Nm2(idx),fliplr(WS_Nm2(idx))], ...
         [PW_envelope_WN(idx),PWplotMax*ones(size(idx))], ...
         [0.88 0.94 0.90], 'FaceAlpha',0.55, 'EdgeColor','none', ...
         'DisplayName','Feasible: thresholds + electric range');
end

PW_takeoff_plot_WN = PW_takeoff_WN;
PW_takeoff_plot_WN(~takeoffGeometryOK) = NaN;
plot(WS_Nm2,PW_takeoff_plot_WN,'LineWidth',1.8,'DisplayName','Physics takeoff <= 91.44 m');
plot(WS_Nm2,PW_RFP_ROC_WN,'LineWidth',1.8,'DisplayName','RFP initial climb >= 7.62 m/s');
plot(WS_Nm2,PW_initial_WN,'LineWidth',1.8,'DisplayName','FAA/EASA initial climb >= 4%');
plot(WS_Nm2,PW_CLoT_WN,'LineWidth',1.8,'DisplayName',sprintf('FAA/EASA CLoT climb >= %.0f%%',100*G_CLoT));
plot(WS_Nm2,PW_balked_WN,'LineWidth',1.8,'DisplayName','FAA/EASA balked landing >= 3%');
plot(WS_Nm2,PW_cruise_225_WN,'LineWidth',1.8,'DisplayName','RFP cruise >= 115.75 m/s');

if use_maneuver_constraint
    PW_maneuver_plot_WN = PW_maneuver_WN;
    PW_maneuver_plot_WN(~maneuverLiftOK) = NaN;
    plot(WS_Nm2,PW_maneuver_plot_WN,'Color',[0 1 0],'LineWidth',1.8, ...
        'DisplayName',sprintf('Sustained %.0f deg bank @ %.0f kt',phi_maneuver_deg,V_maneuver_ms/0.514444));
end

if use_ceiling_constraint
    plot(WS_Nm2,PW_ceiling_WN,'Color',[1 0.5 0.8],'LineWidth',1.8, ...
        'DisplayName',sprintf('Service ceiling %.1f kft / %.1f fpm',h_ceiling_m/0.3048/1000,ROC_ceiling_ms/0.00508));
end

plot([WS_landing_max_Nm2 WS_landing_max_Nm2],[0 PWplotMax],'k','LineWidth',1.8,'DisplayName','RFP landing <= 91.44 m');

if combinedFeasibleExists && electricRangePassSelected
    plot(WS_design_Nm2, PW_design_WN,'kp','MarkerSize',14,'MarkerFaceColor','y','DisplayName','Selected design');
else
    plot(WS_design_Nm2, PW_design_WN,'rp','MarkerSize',14,'MarkerFaceColor','r','DisplayName','Diagnostic point');
end

text(WS_design_Nm2, PW_design_WN,sprintf(' W/S = %.0f N/m^2\n P/W = %.2f W/N',WS_design_Nm2,PW_design_WN),'FontWeight','bold');

xlabel('Takeoff Wing Loading, W/S [N/m^2]');
ylabel('Installed Power-to-Weight, P/W [W/N]');
title('M-BRAER Preliminary P/W - W/S Constraint Diagram (SI)');
xlim([200, min(max(WS_Nm2),1.20*WS_vertical_limit_Nm2)]);
ylim([0,PWplotMax]);
legend('Location','best');

exportFigure('AE481_Assignment4_PW_WS_SI');

%% ========================================================================
% LOCAL FUNCTIONS - ALL SI
% ========================================================================

function rho = atmDensitySI(h_m,dT_C)
    T0 = 288.15; p0 = 101325; L = 0.0065; g0 = 9.80665; R = 287.05287;
    Tisa = T0 - L*h_m;
    p = p0*(Tisa/T0)^(g0/(R*L));
    Tactual = Tisa + dT_C;
    rho = p/(R*Tactual);
end

function Vstall_ms = stallSpeedSI(WS_TO_Nm2,betaW,rho,CLmax)
    WS_local = betaW*WS_TO_Nm2;
    Vstall_ms = sqrt(2*WS_local/(rho*CLmax));
end

function WSmax_TO_Nm2 = stallWsLimitSI(Vstall_ms,rho,CLmax,betaW)
    WS_condition = 0.5*rho*Vstall_ms^2*CLmax;
    WSmax_TO_Nm2 = WS_condition/betaW;
end

function [PWinstalled_WN,geometryOK,sg_m,str_m,scl_m,htr_m] = physicsTakeoffPW_SI( ...
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
    str_m = zeros(size(WS_Nm2)); htr_m = zeros(size(WS_Nm2)); scl_m = zeros(size(WS_Nm2));
    str_m(belowObstacle) = str_full_m(belowObstacle);
    htr_m(belowObstacle) = htr_full_m(belowObstacle);
    scl_m(belowObstacle) = (hObstacle_m - htr_full_m(belowObstacle))./tan(gamma);
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
    aeroGroundTerm = (kR^2/(2*CLmax)).*(CD_TO - mu*CL_TO);
    TW_required = (kR^2.*WS_Nm2)./(g0*rho*CLmax.*sg_m) + mu + aeroGroundTerm;
    PWavailable_WN = TW_required.*Vtr_ms./etaProp;
    PWinstalled_WN = PWavailable_WN./powerAvailFrac;
    PWinstalled_WN(~geometryOK) = inf;
end

function [sTotal_m,sg_m,str_m,scl_m,htr_m] = physicsTakeoffDistance_SI( ...
    WS_Nm2,PWinstalled_WN,hObstacle_m,rho,CLmax,CL_TO,CD_TO,mu,etaProp, ...
    powerAvailFrac,kR,kTr,nTransition,gamma_deg)
    g0 = 9.80665;
    gamma = deg2rad(gamma_deg);
    Vs_ms  = sqrt(2.*WS_Nm2./(rho*CLmax));
    Vtr_ms = kTr.*Vs_ms;
    PWavailable_WN = PWinstalled_WN.*powerAvailFrac;
    TW_available = etaProp.*PWavailable_WN./Vtr_ms;
    aeroGroundTerm = (kR^2/(2*CLmax)).*(CD_TO - mu*CL_TO);
    accelBracket = TW_available - mu - aeroGroundTerm;
    if any(accelBracket <= 0)
        sg_m = inf(size(WS_Nm2));
    else
        sg_m = (kR^2.*WS_Nm2)./(g0*rho*CLmax.*accelBracket);
    end
    R_m = Vtr_ms.^2./(g0*(nTransition - 1));
    str_full_m = R_m.*sin(gamma);
    htr_full_m = R_m.*(1 - cos(gamma));
    if htr_full_m < hObstacle_m
        str_m = str_full_m; htr_m = htr_full_m; scl_m = (hObstacle_m - htr_m)./tan(gamma);
    else
        thetaObs = acos(1 - hObstacle_m./R_m);
        str_m = R_m.*sin(thetaObs); htr_m = hObstacle_m; scl_m = 0;
    end
    sTotal_m = sg_m + str_m + scl_m;
end

function [sTotal_m, sg_m, sfl_m, sapp_m, hfl_m] = landingDistanceExtremeSTOL_SI( ...
    WS_TO_Nm2, betaW, rho, CLmax, gamma_deg, kTD, decel_g, hObstacle_m, nFlare, safetyFactor)
    if nargin < 9 || isempty(nFlare), nFlare = 1.15; end
    if nargin < 10 || isempty(safetyFactor), safetyFactor = 1.0; end
    g0 = 9.80665; gamma = deg2rad(gamma_deg);
    WS_L_Nm2 = betaW .* WS_TO_Nm2;
    Vs_ms = sqrt(2 .* WS_L_Nm2 ./ (rho * CLmax));
    VTD_ms = kTD .* Vs_ms;
    R_m = VTD_ms.^2 ./ (g0 * (nFlare - 1));
    sfl_full_m = R_m .* sin(gamma); hfl_full_m = R_m .* (1 - cos(gamma));
    belowObstacle = hfl_full_m < hObstacle_m;
    sfl_m = zeros(size(WS_TO_Nm2)); hfl_m = zeros(size(WS_TO_Nm2)); sapp_m = zeros(size(WS_TO_Nm2));
    sfl_m(belowObstacle) = sfl_full_m(belowObstacle);
    hfl_m(belowObstacle) = hfl_full_m(belowObstacle);
    sapp_m(belowObstacle) = (hObstacle_m - hfl_full_m(belowObstacle)) ./ tan(gamma);
    reachesObstacleInFlare = ~belowObstacle;
    if any(reachesObstacleInFlare)
        Rlocal = R_m(reachesObstacleInFlare);
        thetaObs = acos(1 - hObstacle_m ./ Rlocal);
        sfl_m(reachesObstacleInFlare) = Rlocal .* sin(thetaObs);
        hfl_m(reachesObstacleInFlare) = hObstacle_m;
        sapp_m(reachesObstacleInFlare) = 0;
    end
    aStop_ms2 = decel_g * g0;
    sg_m = VTD_ms.^2 ./ (2 * aStop_ms2);
    sTotal_m = (sg_m + sfl_m + sapp_m) .* safetyFactor;
end

function PW_TO_WN = climbPWfromGradient_SI(WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,CLmax,ks,G,powerRemaining,powerAvailFrac)
    WS_local_Nm2 = betaW.*WS_TO_Nm2;
    CL = CLmax/ks^2;
    V_ms = sqrt(2.*WS_local_Nm2./(rho*CL));
    K = 1/(pi*AR*e);
    CD = CD0 + K*CL^2;
    D_over_W = CD./CL;
    PW_current_WN = V_ms./eta .* (D_over_W + G);
    PW_TO_WN = betaW.*PW_current_WN ./ (powerRemaining*powerAvailFrac);
end

function PW_TO_WN = climbPWfromROC_SI(WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,CLmax,ks,ROC_ms,powerRemaining,powerAvailFrac)
    WS_local_Nm2 = betaW.*WS_TO_Nm2;
    CL = CLmax/ks^2;
    V_ms = sqrt(2.*WS_local_Nm2./(rho*CL));
    G = ROC_ms./V_ms;
    PW_TO_WN = climbPWfromGradient_SI(WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,CLmax,ks,G,powerRemaining,powerAvailFrac);
end

function PW_TO_WN = cruisePW_SI(WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,V_ms,powerAvailFrac)
    q_Pa = 0.5*rho*V_ms^2;
    K = 1/(pi*AR*e);
    parasiteTerm = q_Pa*CD0./WS_TO_Nm2;
    inducedTerm = K*betaW^2.*WS_TO_Nm2./q_Pa;
    PW_TO_WN = V_ms/(eta*powerAvailFrac) .* (parasiteTerm + inducedTerm);
end

function [PW_TO_WN,liftOK] = maneuverPW_SI(WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,CLmax,V_ms,n,powerAvailFrac)
    q_Pa = 0.5*rho*V_ms^2;
    WS_maneuver_Nm2 = betaW.*WS_TO_Nm2;
    TW_maneuver = q_Pa.*CD0./WS_maneuver_Nm2 + WS_maneuver_Nm2.*n^2./(q_Pa*pi*AR*e);
    PW_TO_WN = betaW.*TW_maneuver.*V_ms./(eta*powerAvailFrac);
    CL = n.*WS_maneuver_Nm2./q_Pa;
    liftOK = CL <= CLmax;
    PW_TO_WN(~liftOK) = inf;
end

function [PW_TO_WN,Vbest_ms,CLbest] = serviceCeilingPW_SI(WS_TO_Nm2,betaW,rho,eta,CD0,e,AR,CLmax,ksMin,ROC_req_ms,powerAvailFrac)
    K = 1/(pi*AR*e);
    CL_minPower = sqrt(3.*CD0./K);
    CL_stallMargin = CLmax/(ksMin^2);
    CLbest = min(CL_minPower,CL_stallMargin);
    WS_local_Nm2 = betaW.*WS_TO_Nm2;
    Vbest_ms = sqrt(2.*WS_local_Nm2./(rho.*CLbest));
    CD = CD0 + K.*CLbest.^2;
    D_over_W_current = CD./CLbest;
    PW_TO_WN = betaW.*(Vbest_ms.*D_over_W_current + ROC_req_ms) ./ (eta.*powerAvailFrac);
end

function phi_deg = maxSustainedBank_SI(WS_TO_Nm2,PWinstalled_WN,betaW,rho,eta,CD0,e,AR,CLmax,V_ms,powerAvailFrac)
    q_Pa = 0.5*rho*V_ms^2;
    K = 1/(pi*AR*e);
    CD_power = eta*PWinstalled_WN*powerAvailFrac*WS_TO_Nm2/(q_Pa*V_ms);
    inducedAllowance = max(CD_power - CD0,0);
    if inducedAllowance <= 0
        phi_deg = 0; return;
    end
    n_power = sqrt(inducedAllowance*q_Pa^2 / (K*(betaW*WS_TO_Nm2)^2));
    n_lift = q_Pa*CLmax/(betaW*WS_TO_Nm2);
    n_max = min(n_power,n_lift);
    if n_max <= 1, phi_deg = 0; else, phi_deg = acosd(1/n_max); end
end

function [ROCmax_ms,Vbest_ms] = bestROC_SI(WS_TO_Nm2,PWinstalled_WN,betaW,rho,eta,CD0,e,AR,CLmax,powerAvailFrac,powerRemaining)
    WS_local_Nm2 = betaW*WS_TO_Nm2;
    Vs_ms = sqrt(2*WS_local_Nm2/(rho*CLmax));
    V_ms = linspace(1.05*Vs_ms,140,2500);
    q_Pa = 0.5*rho.*V_ms.^2;
    CL = WS_local_Nm2./q_Pa;
    K = 1/(pi*AR*e);
    CD = CD0 + K.*CL.^2;
    valid = CL <= CLmax;
    D_over_W = CD./CL;
    PW_current_WN = PWinstalled_WN*powerAvailFrac*powerRemaining/betaW;
    specificPowerAvailable_ms = eta*PW_current_WN;
    ROC_ms = specificPowerAvailable_ms - D_over_W.*V_ms;
    ROC_ms(~valid) = -inf;
    [ROCmax_ms,idx] = max(ROC_ms);
    Vbest_ms = V_ms(idx);
end

function [R_km,Vbest_ms,LDmax] = electricEngineOutRange_SI(W_N,WS_TO_Nm2,rho,CD0,e,AR,mBattery_kg,specificEnergy_Whkg,usableFraction,etaTotal)
    K = 1/(pi*AR*e);
    CL_LDmax = sqrt(CD0/K);
    LDmax = CL_LDmax/(CD0 + K*CL_LDmax^2);
    Vbest_ms = sqrt(2*WS_TO_Nm2/(rho*CL_LDmax));
    E_usable_Wh = mBattery_kg*specificEnergy_Whkg*usableFraction;
    E_propulsive_J = E_usable_Wh*3600*etaTotal;
    D_N = W_N/LDmax;
    R_m = E_propulsive_J/D_N;
    R_km = R_m/1000;
end

function exportFigure(baseName)
    try
        exportgraphics(gcf,[baseName '.pdf'],'ContentType','vector');
        exportgraphics(gcf,[baseName '.png'],'Resolution',600);
    catch
        set(gcf,'PaperPositionMode','auto');
        print(gcf,[baseName '.pdf'],'-dpdf','-painters');
        print(gcf,[baseName '.png'],'-dpng','-r600');
    end
end