%% AE481 - PARAMETRIC P-S CONSTRAINT DIAGRAM (KODIAK 100 COMPARISON)
clear; clc; close all;

%% ========================================================================
% 1. CURRENT BASELINE FROM WEIGHT CODE
% ========================================================================
if exist('Weight_Predict.m','file') ~= 2
    error('Weight_Predict.m was not found in the path.');
end
if exist('Weight_Predict_PS.m','file') ~= 2
    error('Weight_Predict_PS.m must be in the same MATLAB folder.');
end

% Call Kodiak 100 baseline: 450 nmi range, 174 kt cruise, 10,000 ft, L/D=12.5, 9 people, conventional
baseline = Weight_Predict(450, 174, 10000, 12.5, 9, "conventional", 0);

mTO_baseline_kg       = baseline{'kg','GrossTakeoff'};
mEmpty_baseline_kg    = baseline{'kg','Empty'};
mElectric_baseline_kg = baseline{'kg','ElectricHardware'};

%% ========================================================================
% 2. PARAMETRIC INPUTS / KODIAK 100 AIRCRAFT DATA
% ========================================================================
p.g0 = 9.80665;

% Mission Specs
p.range_nmi         = 450;
p.electricRange_nmi = 0;           % Pure conventional turboprop
p.cruise_kt         = 174;
p.altitude_ft       = 10000;
p.alternate_nmi     = 50;
p.reserve_min       = 45;
p.reserve_kt        = 0.80*p.cruise_kt;
p.engineOut_nmi     = 0;
p.engineOut_kt      = 101;

p.climb_kt      = 101;
p.climb_ftmin   = 1340;
p.descent_kt    = 130;
p.descent_ftmin = 1000;
p.climbDistance_nmi = p.climb_kt*(p.altitude_ft/p.climb_ftmin)/60;
p.descentDistance_nmi = p.descent_kt*(p.altitude_ft/p.descent_ftmin)/60;

% Payload: 1 pilot + 8 passengers = 9 total
num_people = 9;
p.mPeople_kg  = 86.1825*num_people;
p.mBaggage_kg = 13.6075*num_people;

% Aerodynamics / Kodiak 100 Wing Geometry
p.AR       = 7.4;                  % Aspect Ratio
p.e_clean  = 0.78;
p.e_TO     = 0.73;
p.e_L      = 0.68;
p.e_CLoT   = 0.70;
p.CD0_ref  = 0.026;                % Fixed gear / high wing drag baseline
p.Cf_equiv = 0.0035;
p.LD_ref   = 12.5;                 % Cruise L/D estimate for STOL utility

p.CLmax_clean = 1.60;
p.CLmax_TO    = 2.20;
p.CLmax_L     = 2.80;
p.dCD0_TO_flaps = 0.020;
p.dCD0_L_flaps  = 0.075;
p.dCD0_gear     = 0.015;           % Fixed landing gear present
p.dCD0_CLoT     = 0.010;

% Weight Fractions
p.beta_cruise   = 0.95;
p.beta_landing  = 0.98;
p.beta_climb    = 0.99;
p.beta_maneuver = 0.95;
p.beta_ceiling  = 0.95;

% Propeller Efficiencies
p.eta_TO       = 0.75;
p.eta_climb    = 0.80;
p.eta_cruise   = 0.82;
p.eta_balked   = 0.75;
p.eta_maneuver = 0.82;
p.eta_ceiling  = 0.80;

p.powerFrac_TO       = 1.00;
p.powerFrac_climb    = 1.00;
p.powerFrac_cruise   = 0.75;
p.powerFrac_balked   = 1.00;
p.powerFrac_maneuver = 0.80;
p.powerFrac_ceiling  = 0.75;

% Propulsion Architecture Parameters
p.thermalShare  = 1.00;            % 100% Turboprop shaft power
p.electricShare = 0.00;
p.engineOfftakeFraction = 0.02;

p.hubMotorFractionOfElectric = 0.0;
p.distributedFractionOfElectric = 0.0;
p.numDistributedMotors = 0;
p.generatorFractionOfElectric = 0.0;
p.generator_kWkg = 8.0;
p.generatorInstallationFraction = 0.15;

p.etaMotor    = 0.96;
p.etaInverter = 0.98;
p.etaCable    = 0.996;
p.etaBattery  = 0.96;
p.motor_kWkg    = 10;
p.inverter_kWkg = 16;
p.electricInstallationFraction = 0.30;
p.extraElectricHardware_kg = 0;

p.pack_Whkg = 250;
p.usableSOC = 0.85;
p.pack_kWkg = 1.5;
p.boost_min = 0;
p.auxiliary_kW = 0;

p.Cp_lb_hphr = 0.53;                % PT6A SFC
p.etaProp = 0.82;
p.K_Breguet = 325.9;
p.rWarmup  = 0.990;
p.rTaxi    = 0.995;
p.rTakeoff = 0.995;
p.rClimb   = 0.985;
p.rDescent = 0.985;
p.rLanding = 0.995;

p.Nz_ultimate = 5.7;
p.tc_root = 0.15;
p.taperRatio = 1.0;                 % Un-tapered rectangular wing
p.sweep25_deg = 0;
p.controlSurfaceFraction = 0.25;

p.kPropulsionGroup = 1.16;

p.weightTolerance = 1e-7;
p.maxWeightIterations = 1000;
p.massNumericalLimit_kg = 10000;
p.mTO_baseline_kg = mTO_baseline_kg;
p.mEmpty_baseline_kg = mEmpty_baseline_kg;
p.mElectric_baseline_kg = mElectric_baseline_kg;

% Performance & STOL Operational Constraints
p.Vcruise225_ms = 174*0.514444;    % Target cruise speed: 174 kt
p.Vcruise250_ms = 185*0.514444;    % Objective speed line
p.sTOreq_m = 310;                  % Takeoff distance requirement (~1000 ft STOL)
p.sLreq_m  = 300;                  % Landing distance requirement (~980 ft)
p.hObstacle_m = 50*0.3048;
p.ROC_RFP_ms = 1340*0.00508;       % Rate of Climb requirement

p.h_field_m = 0;
p.dT_field_C = 0;
p.h_cruise_m = 10000*0.3048;
p.dT_cruise_C = 0;
p.h_ceiling_m = 25000*0.3048;
p.dT_ceiling_C = 0;
p.ROC_ceiling_ms = 100*0.00508;

p.G_initial = 0.04;
p.G_CLoT    = 0.01;
p.G_balked  = 0.03;
p.CLoT_power_remaining = 0.50;

p.ks_initial = 1.20;
p.ks_CLoT    = 1.20;
p.ks_balked  = 1.30;
p.ks_ceiling = 1.10;

p.mu_roll = 0.04;                  % Grass/unpaved surface rolling friction
p.k_rotation = 1.10;
p.k_transition = 1.15;
p.n_transition = 1.30;
p.gamma_takeoff_deg = 10;

p.gamma_approach_deg = 8;
p.k_touchdown = 1.15;
p.stop_decel_g = 0.40;
p.n_flare = 1.15;

p.phi_maneuver_deg = 45;            % Utility bank angle standard
p.n_maneuver = 1/cosd(p.phi_maneuver_deg);
p.V_maneuver_ms = p.Vcruise225_ms;

p.MTOW_limit_kg = 7255*0.45359237;  % Kodiak 100 Max Takeoff Mass (3,290 kg)

% Atmosphere
p.rho_field   = atmDensityPS(p.h_field_m,p.dT_field_C);
p.rho_cruise  = atmDensityPS(p.h_cruise_m,p.dT_cruise_C);
p.rho_ceiling = atmDensityPS(p.h_ceiling_m,p.dT_ceiling_C);

%% ========================================================================
% 3. ENVIRONMENTAL MODEL
% ========================================================================
p.fuelCO2_kg_per_kgFuel = 3.16;
p.gridCO2_kg_per_kWh = 0.348018746;
p.batteryManufacturing_kgCO2e_per_kWh = 64;
p.airframeManufacturing_kgCO2e_per_kgEmpty = 1401164/42092;
p.gridChargeEfficiency = 0.95;
p.lifeMissions = 15000;

%% ========================================================================
% 4. BASELINE GEOMETRY & POWER
% ========================================================================
Wbase_N = mTO_baseline_kg*p.g0;

WS_scan = linspace(50,2000,5000);
sL_scan = landingDistancePS(WS_scan,p);
landingOK = sL_scan <= p.sLreq_m;
if ~any(landingOK)
    error('No landing-feasible W/S found in the scan.');
end

WS_landing_max = max(WS_scan(landingOK));
WS_ref = 0.90*WS_landing_max;
S_ref_m2 = Wbase_N/WS_ref;          % Reference wing area (~22.3 m^2 / 240 ft^2)

p.S_ref_m2 = S_ref_m2;
p.Swet_rest_m2 = (p.CD0_ref/p.Cf_equiv)*S_ref_m2 - 2*S_ref_m2;

baseAero.CD0_clean = p.CD0_ref;
basePerf = performanceConstraintsPS(WS_ref,baseAero.CD0_clean,p);
PW_ref = 1.05*basePerf.PW_threshold_WN;
P_ref_kW = PW_ref*Wbase_N/1000;     % Power baseline (~559 kW / 750 shp)
p.P_ref_kW = P_ref_kW;

%% ========================================================================
% 5. P-S SWEEP GRID
% ========================================================================
engineName = {'PT6A-135A (750 shp)'; 'PT6A-34 (750 shp)'; ...
    'PT6A-140 (867 shp)'; 'GE Catalyst (1300 shp)'};
engineSHP = [750; 750; 867; 1300];
engineRated_kW = engineSHP*0.745699872;

engineNetThermal_kW = engineRated_kW*(1-p.engineOfftakeFraction);
engineSystemPower_kW = engineNetThermal_kW/p.thermalShare;

nS = 81;
nP = 91;
S_vec_m2 = linspace(15, 35, nS);   % Wing Area range [m^2] (~160 to 375 ft^2)
P_vec_kW = linspace(250, 800, nP); % Shaft Power range [kW] (~335 to 1070 hp)

[S_grid_m2,P_grid_kW] = meshgrid(S_vec_m2,P_vec_kW);
sz = size(S_grid_m2);

J_kgCO2eMission = nan(sz);
MTO_kg = nan(sz);
Feasible = false(sz);

M_takeoff = nan(sz);
M_roc = nan(sz);
M_cruise = nan(sz);
M_landing = nan(sz);
M_mtow = nan(sz);

for iP = 1:nP
    for iS = 1:nS
        d = evaluatePSPoint(S_grid_m2(iP,iS),P_grid_kW(iP,iS),p);
        if ~d.valid, continue; end

        J_kgCO2eMission(iP,iS) = d.objective_kgCO2e_perMission;
        MTO_kg(iP,iS) = d.mass.mTO_kg;
        Feasible(iP,iS) = d.feasible;

        M_takeoff(iP,iS) = d.margin_takeoff_kW;
        M_roc(iP,iS) = d.margin_roc_kW;
        M_cruise(iP,iS) = d.margin_cruise225_kW;
        M_landing(iP,iS) = d.margin_landing_m;
        M_mtow(iP,iS) = d.margin_mtow_kg;
    end
end

%% ========================================================================
% 6. PLOT: T-S CONSTRAINT DIAGRAM
% ========================================================================
figure('Color','w','Position',[70 70 1100 700]);
hold on; grid on; box on;

% Shading for Feasible Region
if any(Feasible(:))
    [~,hFeas] = contourf(S_grid_m2, P_grid_kW, double(Feasible), [0.5 1.5], 'LineStyle', 'none');
    hFeas.FaceAlpha = 0.20;
    hFeas.DisplayName = 'Feasible Design Space';
end

% Constraints
cc = lines(10);
plotZeroContour(S_grid_m2, P_grid_kW, M_takeoff, 'Takeoff <= 1000 ft', 1.8, cc(1,:));
plotZeroContour(S_grid_m2, P_grid_kW, M_roc, 'ROC >= 1340 ft/min', 1.8, cc(2,:));
plotZeroContour(S_grid_m2, P_grid_kW, M_cruise, 'Cruise >= 174 kt', 2.0, cc(6,:));
plotZeroContour(S_grid_m2, P_grid_kW, M_landing, 'Landing <= 980 ft', 2.0, [1 0.5 0]);
plotZeroContour(S_grid_m2, P_grid_kW, M_mtow, 'MTOW <= 7,255 lb', 1.8, cc(10,:));

% Overlay Real Kodiak 100 Aircraft Operating Point (22.3 m^2 wing area, 559 kW installed power)
plot(22.3, 559, 'kp', 'MarkerSize', 16, 'MarkerFaceColor', 'y', 'DisplayName', 'Kodiak 100 Real Point (22.3 m^2, 559 kW)');

xlabel('Wing Reference Area, S [m^2]');
ylabel('Total Installed Shaft Power, P [kW]');
title('P-S Constraint Diagram Anchored to Kodiak 100 Aircraft Parameters');
legend('Location', 'northeast');

exportFigurePS('Kodiak100_PS_Constraint_Diagram');

%% ========================================================================
% HELPER FUNCTIONS
% ========================================================================
function d = evaluatePSPoint(S_m2,P_kW,p)
    d.valid = false;
    d.S_m2 = S_m2;
    d.P_kW = P_kW;

    m = Weight_Predict_PS(S_m2,P_kW,p);
    if ~m.valid, return; end

    W_N = m.mTO_kg*p.g0;
    WS_Nm2 = W_N/S_m2;

    c = performanceConstraintsPS(WS_Nm2,m.CD0_clean,p);
    if ~isfinite(c.PW_threshold_WN), return; end

    P_threshold_kW = c.PW_threshold_WN*W_N/1000;
    powerMarginFraction = P_kW/P_threshold_kW - 1;

    feasible = c.geometryOK && (P_kW >= P_threshold_kW) ...
        && (c.sLanding_m <= p.sLreq_m) && (m.mTO_kg <= p.MTOW_limit_kg);

    operation_kgCO2 = p.fuelCO2_kg_per_kgFuel*m.mFuel_kg;
    manufacturing_kgCO2e = p.airframeManufacturing_kgCO2e_per_kgEmpty*m.mEmpty_kg;
    objective = operation_kgCO2 + manufacturing_kgCO2e/p.lifeMissions;

    d.valid = true;
    d.mass = m;
    d.constraints = c;
    d.feasible = feasible;
    d.margin_takeoff_kW = P_kW - (c.PW_takeoff_WN*W_N/1000);
    d.margin_roc_kW = P_kW - (c.PW_roc_WN*W_N/1000);
    d.margin_cruise225_kW = P_kW - (c.PW_cruise225_WN*W_N/1000);
    d.margin_landing_m = p.sLreq_m - c.sLanding_m;
    d.margin_mtow_kg = p.MTOW_limit_kg - m.mTO_kg;
    d.objective_kgCO2e_perMission = objective;
end

function c = performanceConstraintsPS(WS_Nm2,CD0clean,p)
    K_TO = 1/(pi*p.AR*p.e_TO);
    CL_ground = p.CLmax_TO/p.k_rotation^2;
    CD_ground = CD0clean + p.dCD0_TO_flaps + p.dCD0_gear + K_TO*CL_ground^2;

    [c.PW_takeoff_WN,c.geometryOK] = physicsTakeoffPW_PS( ...
        WS_Nm2,p.sTOreq_m,p.hObstacle_m,p.rho_field,p.CLmax_TO, ...
        CL_ground,CD_ground,p.mu_roll,p.eta_TO,p.powerFrac_TO, ...
        p.k_rotation,p.k_transition,p.n_transition,p.gamma_takeoff_deg);

    c.PW_roc_WN = climbPWfromROC_PS( ...
        WS_Nm2,p.beta_climb,p.rho_field,p.eta_climb, ...
        CD0clean+p.dCD0_TO_flaps+p.dCD0_gear,p.e_TO,p.AR, ...
        p.CLmax_TO,p.ks_initial,p.ROC_RFP_ms,1.0,p.powerFrac_climb);

    c.PW_cruise225_WN = cruisePW_PS( ...
        WS_Nm2,p.beta_cruise,p.rho_cruise,p.eta_cruise, ...
        CD0clean,p.e_clean,p.AR,p.Vcruise225_ms,p.powerFrac_cruise);

    c.sLanding_m = landingDistancePS(WS_Nm2,p);

    threshold = [c.PW_takeoff_WN, c.PW_roc_WN, c.PW_cruise225_WN];
    c.PW_threshold_WN = max(threshold);
end

function rho = atmDensityPS(h_m,dT_C)
    T0 = 288.15; p0 = 101325; L = 0.0065; g0 = 9.80665; R = 287.05287;
    Tisa = T0 - L*h_m;
    p = p0*(Tisa/T0)^(g0/(R*L));
    rho = p/(R*(Tisa + dT_C));
end

function [PWinstalled_WN,geometryOK] = physicsTakeoffPW_PS( ...
    WS_Nm2,sTotalReq_m,hObstacle_m,rho,CLmax,CL_TO,CD_TO,mu,etaProp, ...
    powerAvailFrac,kR,kTr,nTransition,gamma_deg)

    g0 = 9.80665;
    gamma = deg2rad(gamma_deg);
    Vs_ms = sqrt(2.*WS_Nm2./(rho*CLmax));
    Vtr_ms = kTr.*Vs_ms;
    R_m = Vtr_ms.^2./(g0*(nTransition - 1));
    str_full_m = R_m.*sin(gamma);
    htr_full_m = R_m.*(1 - cos(gamma));

    str_m = zeros(size(WS_Nm2));
    scl_m = zeros(size(WS_Nm2));
    below = htr_full_m < hObstacle_m;
    str_m(below) = str_full_m(below);
    scl_m(below) = (hObstacle_m - htr_full_m(below))./tan(gamma);

    above = ~below;
    if any(above,'all')
        Rlocal = R_m(above);
        theta = acos(1 - hObstacle_m./Rlocal);
        str_m(above) = Rlocal.*sin(theta);
        scl_m(above) = 0;
    end

    sg_m = sTotalReq_m - str_m - scl_m;
    geometryOK = sg_m > 0;

    aeroGroundTerm = (kR^2/(2*CLmax)).*(CD_TO - mu*CL_TO);
    TW_required = (kR^2.*WS_Nm2)./(g0*rho*CLmax.*sg_m) + mu + aeroGroundTerm;
    PWinstalled_WN = TW_required.*Vtr_ms./etaProp./powerAvailFrac;
    PWinstalled_WN(~geometryOK) = inf;
end

function PW = climbPWfromROC_PS( ...
    WS_TO,betaW,rho,eta,CD0,e,AR,CLmax,ks,ROC,powerRemaining,powerAvailFrac)

    WS_local = betaW.*WS_TO;
    CL = CLmax/ks^2;
    V = sqrt(2.*WS_local./(rho*CL));
    G = ROC./V;
    
    CL_local = CLmax/ks^2;
    K = 1/(pi*AR*e);
    CD = CD0 + K*CL_local^2;
    D_W = CD./CL_local;
    PW_current = V./eta.*(D_W + G);
    PW = betaW.*PW_current./(powerRemaining*powerAvailFrac);
end

function PW = cruisePW_PS(WS_TO,betaW,rho,eta,CD0,e,AR,V,powerAvailFrac)
    q = 0.5*rho*V^2;
    K = 1/(pi*AR*e);
    parasite = q*CD0./WS_TO;
    induced = K*betaW^2.*WS_TO./q;
    PW = V/(eta*powerAvailFrac).*(parasite + induced);
end

function sTotal_m = landingDistancePS(WS_TO,p)
    gamma = deg2rad(p.gamma_approach_deg);
    WS_L = p.beta_landing.*WS_TO;
    Vs = sqrt(2.*WS_L./(p.rho_field*p.CLmax_L));
    VTD = p.k_touchdown.*Vs;

    R = VTD.^2./(p.g0*(p.n_flare - 1));
    sfl_full = R.*sin(gamma);
    hfl_full = R.*(1 - cos(gamma));

    sfl = zeros(size(WS_TO));
    sapp = zeros(size(WS_TO));
    below = hfl_full < p.hObstacle_m;
    sfl(below) = sfl_full(below);
    sapp(below) = (p.hObstacle_m - hfl_full(below))./tan(gamma);

    above = ~below;
    if any(above,'all')
        Rlocal = R(above);
        theta = acos(1 - p.hObstacle_m./Rlocal);
        sfl(above) = Rlocal.*sin(theta);
        sapp(above) = 0;
    end

    sg = VTD.^2./(2*p.stop_decel_g*p.g0);
    sTotal_m = sg + sfl + sapp;
end

function plotZeroContour(X,Y,M,labelText,lineWidth,lineColor)
    if all(~isfinite(M(:))), return; end
    finiteM = M(isfinite(M));
    if isempty(finiteM) || min(finiteM) > 0 || max(finiteM) < 0, return; end
    [~,h] = contour(X,Y,M,[0 0],'LineWidth',lineWidth,'LineColor',lineColor);
    h.DisplayName = labelText;
end

function exportFigurePS(baseName)
    try
        exportgraphics(gcf,[baseName '.pdf'],'ContentType','vector');
        exportgraphics(gcf,[baseName '.png'],'Resolution',600);
    catch
        set(gcf,'PaperPositionMode','auto');
        print(gcf,[baseName '.pdf'],'-dpdf','-painters');
        print(gcf,[baseName '.png'],'-dpng','-r600');
    end
end