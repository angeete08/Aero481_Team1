%% AE481 - PARAMETRIC P-S CONSTRAINT DIAGRAM + ENVIRONMENTAL OBJECTIVE
% M-BRAER Hybrid-Electric STOL Commuter
%
% Architecture represented here:
%   - partial parallel-series hybrid
%   - one central turboprop engine mechanically coupled to the central propulsor
%   - one central electric hub motor on the same propulsor
%   - eight smaller electric propulsors used for distributed/blown lift
%   - generator included for series-mode capability
%
% PURPOSE
%   1) Convert the existing P/W - W/S sizing into dimensional P - S space.
%   2) Make takeoff weight a function W0 = W0(S,P).
%   3) Overlay the performance constraints and shade the feasible region.
%   4) Overlay a simple life-cycle climate objective (manufacturing + operation).
%   5) Show discrete existing-engine choices and select a point with margin.
%
% This script uses the uploaded Weight_Predict.m only to ANCHOR the current
% baseline design. Weight_Predict_PS.m then performs the actual P-S weight
% variation. Keep all three .m files in the same MATLAB folder.
%
% OUTPUTS
%   AE481_PS_Environmental.pdf  (vector)
%   AE481_PS_Environmental.png  (600 dpi)
%   AE481_PS_Engine_Candidates.csv
%   AE481_PS_Selected_Design.csv
%
% IMPORTANT MODELING NOTE
% This is intentionally a preliminary/simplified life-cycle objective, not a
% certification LCA. It covers direct fuel CO2, grid CO2 for battery recharge,
% an airframe-manufacturing proxy, and battery-manufacturing CO2e. It does not
% include NOx/contrails, upstream kerosene production, maintenance, battery
% replacement, or end-of-life credits.

clear; clc; close all;

%% ========================================================================
% 1. CURRENT BASELINE FROM THE TEAM'S WEIGHT CODE
% ========================================================================

if exist('Weight_Predict.m','file') ~= 2
    error(['Weight_Predict.m was not found. Rename the supplied ', ...
           'Weight_Predict file to Weight_Predict.m and place it here.']);
end
if exist('Weight_Predict_PS.m','file') ~= 2
    error('Weight_Predict_PS.m must be in the same MATLAB folder.');
end

baseline = Weight_Predict(400,225,10000,18,8,"electric",210);

mTO_baseline_kg       = baseline{'kg','GrossTakeoff'};
mEmpty_baseline_kg    = baseline{'kg','Empty'};
mElectric_baseline_kg = baseline{'kg','ElectricHardware'};

%% ========================================================================
% 2. ALL PARAMETRIC INPUTS / TEAM ASSUMPTIONS
% ========================================================================

p.g0 = 9.80665;

% Mission
p.range_nmi         = 400;
p.electricRange_nmi = 210;
p.cruise_kt         = 225;
p.altitude_ft       = 10000;
p.alternate_nmi     = 50;
p.reserve_min       = 30;
p.reserve_kt        = 0.80*p.cruise_kt;
p.engineOut_nmi     = 20;
p.engineOut_kt      = 120;

p.climb_kt      = 120;
p.climb_ftmin   = 1500;
p.descent_kt    = 150;
p.descent_ftmin = 1000;
p.climbDistance_nmi = p.climb_kt*(p.altitude_ft/p.climb_ftmin)/60;
p.descentDistance_nmi = p.descent_kt*(p.altitude_ft/p.descent_ftmin)/60;

% Payload: retained from uploaded Weight_Predict.m
num_people = 8;
p.mPeople_kg  = 86.1825*num_people;
p.mBaggage_kg = 13.6075*num_people;

% Aerodynamics / configuration
p.AR       = 6.0;
p.e_clean  = 0.825;
p.e_TO     = 0.775;
p.e_L      = 0.725;
p.e_CLoT   = 0.75;
p.CD0_ref  = 0.018;        % current team's reference clean CD0
p.Cf_equiv = 0.0030;
p.LD_ref   = 18;           % preserve current weight-code L/D at baseline S

p.CLmax_clean = 1.80;
p.CLmax_TO    = 4.0;
p.CLmax_L     = 5.0;
p.dCD0_TO_flaps = 0.015;
p.dCD0_L_flaps  = 0.065;
p.dCD0_gear     = 0.020;
p.dCD0_CLoT     = 0.010;

% Weight fractions
p.beta_cruise  = 0.97;
p.beta_landing = 1.00;
p.beta_climb   = 1.00;
p.beta_maneuver = 0.97;
p.beta_ceiling  = 0.97;

% Propeller efficiencies and available-rated-power fractions
p.eta_TO      = 0.75;
p.eta_climb   = 0.80;
p.eta_cruise  = 0.85;
p.eta_balked  = 0.75;
p.eta_maneuver = 0.85;
p.eta_ceiling  = 0.85;

p.powerFrac_TO       = 1.00;
p.powerFrac_climb    = 1.00;
p.powerFrac_cruise   = 0.80;
p.powerFrac_balked   = 1.00;
p.powerFrac_maneuver = 0.80;
p.powerFrac_ceiling  = 0.70;

% Hybrid architecture - PARAMETRIC TEAM ASSUMPTIONS
% P on the y-axis is TOTAL INSTALLED PROPULSIVE SHAFT POWER.
p.thermalShare  = 0.50;   % engine propulsive share at full boost
p.electricShare = 0.50;   % electric motor share at full boost

% IMPORTANT: engine offtake is explicitly included.
% 5% of engine rated shaft power is reserved for accessories / non-propulsive
% extraction. Therefore engine rated power > thermal propulsive power.
p.engineOfftakeFraction = 0.05;

% Electric architecture split
p.hubMotorFractionOfElectric = 0.20;  % central hub motor
p.distributedFractionOfElectric = 0.80; % shared by 8 blown-lift motors
p.numDistributedMotors = 8;

% Generator gives the architecture partial-series capability. This is a
% preliminary rating; full mission energy-management optimization is deferred.
p.generatorFractionOfElectric = 0.30;
p.generator_kWkg = 8.0;
p.generatorInstallationFraction = 0.15;

% Electric hardware; aligned with the team's current weight-code assumptions
p.etaMotor    = 0.96;
p.etaInverter = 0.98;
p.etaCable    = 0.996;
p.etaBattery  = 0.96;
p.motor_kWkg    = 10;
p.inverter_kWkg = 16;
p.electricInstallationFraction = 0.30;
p.extraElectricHardware_kg = 5;

% Battery: 2035-style team assumption
p.pack_Whkg = 700;
p.usableSOC = 0.80;
p.pack_kWkg = 2.0;
p.boost_min = 5;
p.auxiliary_kW = 3;

% Fuel / Breguet model from current Weight_Predict.m
p.Cp_lb_hphr = 0.40;
p.etaProp = 0.85;
p.K_Breguet = 325.9;
p.rWarmup  = 0.990;
p.rTaxi    = 0.995;
p.rTakeoff = 0.995;
p.rClimb   = 0.985;
p.rDescent = 0.985;
p.rLanding = 0.995;

% Wing-weight differential: Metabook/Raymer Eq. 7.11.
% These are preliminary geometry assumptions and are deliberately editable.
p.Nz_ultimate = 5.7;
p.tc_root = 0.16;
p.taperRatio = 0.50;
p.sweep25_deg = 0;
p.controlSurfaceFraction = 0.30;

% Propulsion group Eq. 7.21: single central tractor installation.
p.kPropulsionGroup = 1.16;

% Weight-solver controls
p.weightTolerance = 1e-7;
p.maxWeightIterations = 1000;
p.massNumericalLimit_kg = 15000;
p.mTO_baseline_kg = mTO_baseline_kg;
p.mEmpty_baseline_kg = mEmpty_baseline_kg;
p.mElectric_baseline_kg = mElectric_baseline_kg;

% RFP / performance constraints
p.Vcruise225_ms = 225*0.514444;
p.Vcruise250_ms = 250*0.514444; % objective only, not threshold feasibility
p.sTOreq_m = 300*0.3048;
p.sLreq_m  = 300*0.3048;
p.hObstacle_m = 50*0.3048;
p.ROC_RFP_ms = 1500*0.00508;

p.h_field_m = 0;
p.dT_field_C = 10;
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

p.mu_roll = 0.02;
p.k_rotation = 1.10;
p.k_transition = 1.15;
p.n_transition = 1.50;
p.gamma_takeoff_deg = 15;

p.gamma_approach_deg = 15;
p.k_touchdown = 1.15;
p.stop_decel_g = 0.65;
p.n_flare = 1.15;

p.phi_maneuver_deg = 60;
p.n_maneuver = 1/cosd(p.phi_maneuver_deg);
p.V_maneuver_ms = p.Vcruise225_ms;

p.MTOW_limit_kg = 19000*0.45359237;

% Atmosphere
p.rho_field   = atmDensityPS(p.h_field_m,p.dT_field_C);
p.rho_cruise  = atmDensityPS(p.h_cruise_m,p.dT_cruise_C);
p.rho_ceiling = atmDensityPS(p.h_ceiling_m,p.dT_ceiling_C);

%% ========================================================================
% 3. ENVIRONMENTAL OBJECTIVE - SIMPLE CLIMATE PROXY
% ========================================================================
% Sources used for the numerical defaults:
%   ICAO ICEC: 3.16 kg CO2 per kg aviation fuel burned.
%   US EPA eGRID 2023: 767.25 lb CO2/MWh national average = 0.3480 kg/kWh.
%   ICCT 2025 / R&D GREET: US NMC811 production = 64 kg CO2e/kWh.
%   Schwarz et al. (Sustainability 2022): ~1,401,164 kg CO2e manufacturing
%     for a 25-year A320-like reference. CeRAS OWE = 42,092 kg, giving the
%     rough scaling proxy 33.29 kg CO2e per kg empty weight.
%
% These values are PARAMETERS, not hidden constants. Change them for a
% sensitivity study or a different grid / battery supply chain.

p.fuelCO2_kg_per_kgFuel = 3.16;
p.gridCO2_kg_per_kWh = 0.348018746;
p.batteryManufacturing_kgCO2e_per_kWh = 64;
p.airframeManufacturing_kgCO2e_per_kgEmpty = 1401164/42092;
p.gridChargeEfficiency = 0.95;

% TEAM ASSUMPTION: amortize one-time manufacturing across 15,000 missions.
% This is equivalent to 600 missions/year over 25 years; keep parametric.
p.lifeMissions = 15000;

%% ========================================================================
% 4. BUILD A BASELINE (Sref, Pref) FROM THE CURRENT P/W-W/S METHOD
% ========================================================================

Wbase_N = mTO_baseline_kg*p.g0;

% Find the 300-ft landing W/S boundary using the current landing model.
WS_scan = linspace(50,1200,5000);
sL_scan = landingDistancePS(WS_scan,p);
landingOK = sL_scan <= p.sLreq_m;
if ~any(landingOK)
    error('No landing-feasible W/S found in the scan.');
end

WS_landing_max = max(WS_scan(landingOK));
WS_ref = 0.95*WS_landing_max;
S_ref_m2 = Wbase_N/WS_ref;

% Calibrate Eq. (4.58) so the current baseline area has CD0 = 0.018.
p.S_ref_m2 = S_ref_m2;
p.Swet_rest_m2 = (p.CD0_ref/p.Cf_equiv)*S_ref_m2 - 2*S_ref_m2;

% Preliminary required P/W at the baseline point.
baseAero.CD0_clean = p.CD0_ref;
basePerf = performanceConstraintsPS(WS_ref,baseAero.CD0_clean,p);
PW_ref = 1.05*basePerf.PW_threshold_WN;
P_ref_kW = PW_ref*Wbase_N/1000;
p.P_ref_kW = P_ref_kW;

fprintf('\n============================================================\n');
fprintf('BASELINE USED TO CENTER P-S SWEEP\n');
fprintf('============================================================\n');
fprintf('Baseline mass        : %.1f kg\n',mTO_baseline_kg);
fprintf('Landing W/S max      : %.1f N/m^2\n',WS_landing_max);
fprintf('Reference W/S        : %.1f N/m^2\n',WS_ref);
fprintf('Reference S          : %.2f m^2\n',S_ref_m2);
fprintf('Reference P          : %.1f kW\n',P_ref_kW);

%% ========================================================================
% 5. P-S SWEEP: EACH POINT HAS A CONVERGED W0(S,P)
% ========================================================================

% Engine catalogue: one central turbine in each hybrid configuration.
% Mechanical shaft ratings, not thermodynamic/ESHP ratings or sums of engines.
% Sources / rating basis:
% Catalyst: GE, 1300-shp version:
% https://www.geaerospace.com/news/articles/product-technology/catalyze-how-400-engineers-put-their-heads-together-and-reinvented
% TPE331-14GR: Honeywell brochure A60-0885-000-000, 1650 shp SLS.
% https://www.twitterman.com/doc/turbines/honeywell-tpe331-14.pdf
% PT6A-67F: EASA IM.E.008 (1268 kW, approximately 1700 shp).
% https://www.easa.europa.eu/en/downloads/7787/en
% Some aircraft installations derate this engine to 1600 shp; use the
% rating applicable to the proposed installation, not its propeller RPM.
% CT7-9B: Saab 340B normal takeoff rating, 1750 shp (exclude 1870-shp APR).
% https://www.caa.co.uk/Documents/Download/3934/ca692457-336e-4190-840e-cd26cf498935/3902
engineName = {'GE Catalyst (1300 shp)'; 'Honeywell TPE331-14GR'; ...
    'Pratt & Whitney PT6A-67F'; 'GE CT7-9B (normal takeoff)'};
engineSHP = [1300; 1650; 1700; 1750];
engineRated_kW = engineSHP*0.745699872;

assert(p.thermalShare > 0 && p.thermalShare < 1 && ...
    abs(p.thermalShare + p.electricShare - 1) < 1e-12, ...
    'Thermal and electric fractions must be positive and sum to one.');
assert(p.engineOfftakeFraction >= 0 && p.engineOfftakeFraction < 1, ...
    'Engine offtake must be between zero and one.');

engineNetThermal_kW = engineRated_kW*(1-p.engineOfftakeFraction);
engineSystemPower_kW = engineNetThermal_kW/p.thermalShare;
engineElectric_kW = p.electricShare*engineSystemPower_kW;
assert(all(abs(engineSystemPower_kW - ...
    engineNetThermal_kW - engineElectric_kW) < 1e-9));

% This is a coordinate conversion of the ENTIRE diagram, including all
% constraints, contours and design points. Never move only the engine lines.
% true: left axis is actual central-turbine rated kW, right axis is total kW.
% false: left axis is total propulsive kW, right axis is turbine rated kW.
plotEngineRatedPower = true;
if plotEngineRatedPower
    plotPowerScale = p.thermalShare/(1-p.engineOfftakeFraction);
else
    plotPowerScale = 1;
end

% Preserve the original area search; include all catalogue power levels.
nS = 81;
nP = 91;
S_vec_m2 = linspace(0.80*S_ref_m2,1.48*S_ref_m2,nS);
P_vec_kW = linspace(min(0.68*P_ref_kW,0.95*min(engineSystemPower_kW)), ...
    max(1.28*P_ref_kW,1.05*max(engineSystemPower_kW)),nP);
[S_grid_m2,P_grid_kW] = meshgrid(S_vec_m2,P_vec_kW);

sz = size(S_grid_m2);
J_kgCO2eMission = nan(sz);
Operation_kgCO2 = nan(sz);
Manufacturing_kgCO2e = nan(sz);
MTO_kg = nan(sz);
Fuel_kg = nan(sz);
Battery_kg = nan(sz);
LD_grid = nan(sz);
PWactual_grid = nan(sz);
PowerMargin = nan(sz);
Feasible = false(sz);

% Constraint margins. Positive = constraint satisfied.
M_takeoff = nan(sz);
M_roc = nan(sz);
M_initial = nan(sz);
M_clot = nan(sz);
M_balked = nan(sz);
M_cruise = nan(sz);
M_maneuver = nan(sz);
M_ceiling = nan(sz);
M_landing = nan(sz);
M_mtow = nan(sz);
M_cruise250 = nan(sz);  % objective curve only

for iP = 1:nP
    for iS = 1:nS
        d = evaluatePSPoint(S_grid_m2(iP,iS),P_grid_kW(iP,iS),p);
        if ~d.valid
            continue;
        end

        J_kgCO2eMission(iP,iS) = d.objective_kgCO2e_perMission;
        Operation_kgCO2(iP,iS) = d.operation_kgCO2_perMission;
        Manufacturing_kgCO2e(iP,iS) = d.manufacturing_kgCO2e;
        MTO_kg(iP,iS) = d.mass.mTO_kg;
        Fuel_kg(iP,iS) = d.mass.mFuel_kg;
        Battery_kg(iP,iS) = d.mass.mBattery_kg;
        LD_grid(iP,iS) = d.mass.LD_cruise;
        PWactual_grid(iP,iS) = d.PW_actual_WN;
        PowerMargin(iP,iS) = d.powerMarginFraction;
        Feasible(iP,iS) = d.feasible;

        M_takeoff(iP,iS) = d.margin_takeoff_kW;
        M_roc(iP,iS) = d.margin_roc_kW;
        M_initial(iP,iS) = d.margin_initial_kW;
        M_clot(iP,iS) = d.margin_clot_kW;
        M_balked(iP,iS) = d.margin_balked_kW;
        M_cruise(iP,iS) = d.margin_cruise225_kW;
        M_maneuver(iP,iS) = d.margin_maneuver_kW;
        M_ceiling(iP,iS) = d.margin_ceiling_kW;
        M_landing(iP,iS) = d.margin_landing_m;
        M_mtow(iP,iS) = d.margin_mtow_kg;
        M_cruise250(iP,iS) = d.margin_cruise250_kW;
    end
end

if ~any(Feasible(:))
    warning(['No feasible P-S grid point was found. Increase the P/S sweep ', ...
             'or revisit the design assumptions.']);
end

%% ========================================================================
% 6. EXISTING ENGINE LINES + REQUIRED WIGGLE ROOM
% ========================================================================
% Catalogue and unit conversions are defined before the grid in Section 5.
% A point is selectable only if the existing performance, landing, MTOW,
% weight-closure and 7% installed-power-margin checks pass.
% This is P-S model feasibility, not verification of hybrid mission operation.

% Assignment says to give the design some wiggle room.
% Default: require 7% installed-power margin above the governing threshold.
designPowerMargin = 0.07;

nEng = numel(engineSHP);
enginePass = false(nEng,1);
engineBestS_m2 = nan(nEng,1);
engineBestObjective = nan(nEng,1);
engineBestPowerMargin = nan(nEng,1);
engineBestLanding_m = nan(nEng,1);
engineBestMTO_kg = nan(nEng,1);
engineMaxMargin = nan(nEng,1);
engineElectricCruiseMargin_pct = nan(nEng,1);
engineThermalCruiseMargin_pct = nan(nEng,1);
engineStatus = repmat("No valid landing/MTOW point in area search",nEng,1);

S_engine = linspace(min(S_vec_m2),max(S_vec_m2),401);
selectedDesign = [];
selectedObjective = inf;
engineFeasibleArea = false(nEng,numel(S_engine));

for iE = 1:nEng
    Pcandidate = engineSystemPower_kW(iE);
    bestJ = inf;
    maxMarginThisEngine = -inf;

    for iS = 1:numel(S_engine)
        d = evaluatePSPoint(S_engine(iS),Pcandidate,p);
        if ~d.valid
            continue;
        end

        if d.margin_landing_m >= 0 && d.margin_mtow_kg >= 0
            maxMarginThisEngine = max(maxMarginThisEngine,d.powerMarginFraction);
        end

        if d.feasible && d.powerMarginFraction >= designPowerMargin
            engineFeasibleArea(iE,iS) = true;
            if d.objective_kgCO2e_perMission < bestJ
                bestJ = d.objective_kgCO2e_perMission;
                bestD = d; %#ok<NASGU>
            end
        end
    end

    if isfinite(maxMarginThisEngine)
        engineMaxMargin(iE) = maxMarginThisEngine;
        engineStatus(iE) = "Insufficient power margin in area search";
    end

    if isfinite(bestJ)
        enginePass(iE) = true;
        engineStatus(iE) = string(sprintf( ...
            'Passes P-S constraints and %.0f%% power margin',100*designPowerMargin));
        % These are ADDITIONAL diagnostics, not hidden changes to the
        % original P-S constraints or the mission/weight model.
        engineElectricCruiseMargin_pct(iE) = ...
            100*bestD.electricCruisePowerMargin;
        engineThermalCruiseMargin_pct(iE) = ...
            100*bestD.thermalCruisePowerMargin;
        engineBestS_m2(iE) = bestD.S_m2;
        engineBestObjective(iE) = bestD.objective_kgCO2e_perMission;
        engineBestPowerMargin(iE) = bestD.powerMarginFraction;
        engineBestLanding_m(iE) = bestD.constraints.sLanding_m;
        engineBestMTO_kg(iE) = bestD.mass.mTO_kg;

        if bestJ < selectedObjective
            selectedObjective = bestJ;
            selectedDesign = bestD;
            selectedEngineIndex = iE; %#ok<NASGU>
        end
    end
end

EngineCandidates = table( ...
    string(engineName),engineSHP,engineRated_kW,engineNetThermal_kW, ...
    engineElectric_kW,engineSystemPower_kW, ...
    100*engineMaxMargin,enginePass,engineBestS_m2, ...
    100*engineBestPowerMargin,engineBestLanding_m,engineBestMTO_kg, ...
    engineBestObjective,engineElectricCruiseMargin_pct, ...
    engineThermalCruiseMargin_pct,engineStatus, ...
    'VariableNames',{ ...
    'Engine','RatedSHP','Rated_kW','NetThermal_kW','ElectricMotors_kW', ...
    'TotalHybridPropulsive_kW', ...
    'MaximumAvailableMargin_pct','PassesRequiredMargin','SelectedWingArea_m2', ...
    'SelectedPowerMargin_pct','LandingDistance_m','MTOW_kg', ...
    'Objective_kgCO2e_perMission','ElectricCruisePowerMargin_pct', ...
    'ThermalCruisePowerMargin_pct','Status'});

writetable(EngineCandidates,'AE481_PS_Engine_Candidates.csv');

%% ========================================================================
% 7. PLOT: FEASIBLE REGION + CONSTRAINTS + ENVIRONMENTAL CONTOURS
% ========================================================================

P_plot_grid = plotPowerScale*P_grid_kW;

figure('Color','w','Position',[70 70 1350 820]);
hold on; grid on; box on;

% Feasible-region shading
if any(Feasible(:))
    [~,hFeas] = contourf(S_grid_m2,P_plot_grid,double(Feasible),[0.5 1.5], ...
        'LineStyle','none');
    hFeas.FaceAlpha = 0.18;
    hFeas.DisplayName = 'Feasible under P-S model';
end

% Objective contours: use only finite values.
finiteJ = J_kgCO2eMission(isfinite(J_kgCO2eMission));
if ~isempty(finiteJ)
    finiteJ = sort(finiteJ);
    iLo = max(1,round(0.10*numel(finiteJ)));
    iHi = min(numel(finiteJ),round(0.90*numel(finiteJ)));
    lo = finiteJ(iLo);
    hi = finiteJ(iHi);
    if hi <= lo
        levels = linspace(min(finiteJ),max(finiteJ),6);
    else
        levels = linspace(lo,hi,8);
    end
    [Cj,hJ] = contour(S_grid_m2,P_plot_grid,J_kgCO2eMission,levels, ...
        'k--','LineWidth',1.0);
    hJ.HandleVisibility = 'off';
    clabel(Cj,hJ,'FontSize',8,'Color','k');
    plot(NaN,NaN,'k--','LineWidth',1.0, ...
        'DisplayName','Environmental objective contours');
end

% Constraint zero-margins. Positive side is feasible.
cc = lines(10);
plotZeroContour(S_grid_m2,P_plot_grid,M_takeoff, ...
    'Takeoff <= 300 ft',1.6,cc(1,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_roc, ...
    'ROC >= 1500 ft/min',1.6,cc(2,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_initial, ...
    'Initial climb >= 4%',1.6,cc(3,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_clot, ...
    'CLoT climb >= 1%',1.6,cc(4,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_balked, ...
    'Balked landing >= 3%',1.6,cc(5,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_cruise, ...
    'Cruise >= 225 kt',2.0,cc(6,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_maneuver, ...
    'Sustained 60 deg bank',2.0,cc(7,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_ceiling, ...
    'Service ceiling 25,000 ft',1.6,cc(8,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_landing, ...
    'Landing <= 300 ft',2.0,cc(9,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_mtow, ...
    'Part 23 MTOW <= 19,000 lb',1.6,cc(10,:));

% 250-kt cruise objective line (not a threshold feasibility requirement).
[C250,h250] = contour(S_grid_m2,P_plot_grid,M_cruise250,[0 0], ...
    ':','LineWidth',1.5,'LineColor',[0.35 0.35 0.35]);
h250.DisplayName = '250-kt cruise objective';

% Plot only candidates that actually pass the 7% P-S design-margin screen.
% Each colored horizontal segment spans ONLY its passing wing areas.
engineColors = lines(nEng);
for iE = 1:nEng
    if ~enginePass(iE)
        continue;
    end
    yEngine = plotPowerScale*engineSystemPower_kW(iE)*ones(size(S_engine));
    yEngine(~engineFeasibleArea(iE,:)) = NaN;
    plot(S_engine,yEngine,'-.','Color',engineColors(iE,:), ...
        'LineWidth',2.0,'DisplayName', ...
        sprintf('%s: %.0f rated kW; %.0f motor kW; %.0f net total kW', ...
        engineName{iE},engineRated_kW(iE),engineElectric_kW(iE), ...
        engineSystemPower_kW(iE)));
    plot(engineBestS_m2(iE),plotPowerScale*engineSystemPower_kW(iE),'o', ...
        'Color',engineColors(iE,:),'MarkerSize',8,'LineWidth',1.4, ...
        'HandleVisibility','off');
end

% Final selected discrete engine point.
if ~isempty(selectedDesign)
    plot(selectedDesign.S_m2,plotPowerScale*selectedDesign.P_kW,'kp', ...
        'MarkerSize',16,'MarkerFaceColor','y', ...
        'DisplayName','Selected P-S design (mission checks separate)');
    text(selectedDesign.S_m2+1.5,plotPowerScale*(selectedDesign.P_kW+25), ...
        sprintf(['P-S selection: %s\nS = %.1f m^2, P_total = %.0f kW\n', ...
        'Power margin = %.1f%%'], ...
        engineName{selectedEngineIndex},selectedDesign.S_m2, ...
        selectedDesign.P_kW,100*selectedDesign.powerMarginFraction), ...
        'FontWeight','bold','BackgroundColor','w','Margin',3);
end

xlabel('Wing Reference Area, S [m^2]');
if plotEngineRatedPower
    ylabel(sprintf('Central Turbine Rated Shaft Power [kW] (%.0f%% thermal share)', ...
        100*p.thermalShare));
else
    ylabel('Total Installed Propulsive Shaft Power, P [kW]');
end
title({'M-BRAER Parametric P-S Constraint Diagram', ...
       'Environmental objective contours = kg CO_2e per mission equivalent'});

text(0.015,0.02, ...
    sprintf(['P-S model feasibility only; electric/thermal cruise power checks reported separately.  ', ...
    'Engine offtake = %.0f%%.  Required design power margin = %.0f%%.'], ...
    100*p.engineOfftakeFraction,100*designPowerMargin), ...
    'Units','normalized','FontSize',9,'BackgroundColor','w');

xlim([min(S_vec_m2) max(S_vec_m2)]);
ylim(plotPowerScale*[min(P_vec_kW) max(P_vec_kW)]);
% Right-hand scale provides the other power definition without changing data.
yyaxis right;
if plotEngineRatedPower
    ylim([min(P_vec_kW) max(P_vec_kW)]);
    ylabel('Total Hybrid Propulsive Shaft Power [kW]');
else
    ylim(p.thermalShare/(1-p.engineOfftakeFraction)* ...
        [min(P_vec_kW) max(P_vec_kW)]);
    ylabel('Equivalent Central Turbine Rated Shaft Power [kW]');
end
yyaxis left;
legend('Location','eastoutside');

exportFigurePS('AE481_PS_Environmental');

%% ========================================================================
% 8. SELECTED DESIGN / ARCHITECTURE REPORT
% ========================================================================

fprintf('\n============================================================\n');
fprintf('DISCRETE ENGINE CANDIDATES\n');
fprintf('============================================================\n');
disp(EngineCandidates);

if isempty(selectedDesign)
    warning(['No existing-engine candidate satisfies the %.0f%% power margin ', ...
             'inside the current sweep. Reduce the requested margin only if ', ...
             'you can defend it, or add a documented engine rating.'], ...
             100*designPowerMargin);
else
    engName = engineName{selectedEngineIndex};
    PengRated_kW = engineRated_kW(selectedEngineIndex);
    PthermalProp_kW = p.thermalShare*selectedDesign.P_kW;
    Pofftake_kW = p.engineOfftakeFraction*PengRated_kW;
    Pelectric_kW = p.electricShare*selectedDesign.P_kW;
    Phub_kW = p.hubMotorFractionOfElectric*Pelectric_kW;
    PdistributedTotal_kW = p.distributedFractionOfElectric*Pelectric_kW;
    PeachDistributed_kW = PdistributedTotal_kW/p.numDistributedMotors;
    Pgenerator_kW = p.generatorFractionOfElectric*Pelectric_kW;

    fprintf('\n============================================================\n');
    fprintf('SELECTED P-S DESIGN\n');
    fprintf('============================================================\n');
    fprintf('P-S model engine choice  : %s\n',engName);
    fprintf('Engine rated power      : %.1f kW (%.0f shp)\n', ...
        PengRated_kW,engineSHP(selectedEngineIndex));
    fprintf('Engine offtake          : %.1f kW (%.1f%% of rated)\n', ...
        Pofftake_kW,100*p.engineOfftakeFraction);
    fprintf('Thermal propulsive share: %.1f kW\n',PthermalProp_kW);
    fprintf('Total installed P       : %.1f kW\n',selectedDesign.P_kW);
    fprintf('Electric-only cruise shaft margin (optimistic full motor rating): %.1f%%\n', ...
        100*selectedDesign.electricCruisePowerMargin);
    fprintf('Thermal-only cruise shaft margin (existing 80%% availability): %.1f%%\n', ...
        100*selectedDesign.thermalCruisePowerMargin);
    if selectedDesign.electricCruisePowerMargin < 0 || ...
            selectedDesign.thermalCruisePowerMargin < 0
        warning(['The selected point passes the P-S diagram but fails a ', ...
            'separate electric-only or thermal-only cruise power check. ', ...
            'Do not treat it as a verified full-mission propulsion selection. ', ...
            'Motor continuous ratings, engine altitude ratings and mission ', ...
            'energy management need to be reconciled.']);
    end
    fprintf('Wing area S             : %.2f m^2\n',selectedDesign.S_m2);
    fprintf('Converged MTOW          : %.1f kg (%.0f lb)\n', ...
        selectedDesign.mass.mTO_kg,selectedDesign.mass.mTO_kg/0.45359237);
    fprintf('W/S                     : %.1f N/m^2\n',selectedDesign.WS_Nm2);
    fprintf('Required P/W            : %.2f W/N\n', ...
        selectedDesign.constraints.PW_threshold_WN);
    fprintf('Available P/W           : %.2f W/N\n',selectedDesign.PW_actual_WN);
    fprintf('Installed-power margin  : %.1f%%\n', ...
        100*selectedDesign.powerMarginFraction);
    fprintf('Landing distance        : %.2f m\n',selectedDesign.constraints.sLanding_m);
    fprintf('Fuel mass               : %.1f kg\n',selectedDesign.mass.mFuel_kg);
    fprintf('Battery mass            : %.1f kg\n',selectedDesign.mass.mBattery_kg);
    fprintf('Battery nominal energy  : %.1f kWh\n',selectedDesign.mass.EpackNominal_kWh);
    fprintf('Cruise L/D model        : %.2f\n',selectedDesign.mass.LD_cruise);

    fprintf('\nHybrid architecture at selected point:\n');
    fprintf('  Central hub motor     : %.1f kW\n',Phub_kW);
    fprintf('  8 blown-lift motors   : %.1f kW each (%.1f kW total)\n', ...
        PeachDistributed_kW,PdistributedTotal_kW);
    fprintf('  Series generator      : %.1f kW rating assumption\n',Pgenerator_kW);

    fprintf('\nEnvironmental objective:\n');
    fprintf('  Operating CO2         : %.1f kg/mission\n', ...
        selectedDesign.operation_kgCO2_perMission);
    fprintf('  Manufacturing GWP     : %.0f kg CO2e one-time\n', ...
        selectedDesign.manufacturing_kgCO2e);
    fprintf('  Manufacturing amort.  : %.2f kg CO2e/mission\n', ...
        selectedDesign.manufacturing_kgCO2e/p.lifeMissions);
    fprintf('  Objective total       : %.1f kg CO2e/mission equivalent\n', ...
        selectedDesign.objective_kgCO2e_perMission);

    SelectedDesign = table( ...
        string(engName),selectedDesign.S_m2,selectedDesign.P_kW, ...
        selectedDesign.mass.mTO_kg,selectedDesign.WS_Nm2, ...
        selectedDesign.PW_actual_WN,selectedDesign.constraints.PW_threshold_WN, ...
        100*selectedDesign.powerMarginFraction, ...
        selectedDesign.constraints.sLanding_m, ...
        selectedDesign.mass.mFuel_kg,selectedDesign.mass.mBattery_kg, ...
        selectedDesign.mass.EpackNominal_kWh,selectedDesign.mass.LD_cruise, ...
        Phub_kW,PeachDistributed_kW,Pgenerator_kW, ...
        selectedDesign.operation_kgCO2_perMission, ...
        selectedDesign.manufacturing_kgCO2e, ...
        selectedDesign.objective_kgCO2e_perMission, ...
        PengRated_kW,PthermalProp_kW,Pelectric_kW, ...
        100*selectedDesign.electricCruisePowerMargin, ...
        100*selectedDesign.thermalCruisePowerMargin, ...
        (selectedDesign.electricCruisePowerMargin >= 0 && ...
         selectedDesign.thermalCruisePowerMargin >= 0), ...
        'VariableNames' ,{ ...
        'Engine','WingArea_m2','InstalledPower_kW','MTOW_kg','WS_Nm2', ...
        'PWavailable_WN','PWrequired_WN','PowerMargin_pct','LandingDistance_m', ...
        'Fuel_kg','Battery_kg','BatteryNominal_kWh','Cruise_LD', ...
        'HubMotor_kW','EachBlownLiftMotor_kW','Generator_kW', ...
        'OperatingCO2_kgMission','ManufacturingCO2e_kg', ...
        'ObjectiveCO2e_kgMission','EngineRated_kW','NetThermal_kW', ...
        'ElectricMotors_kW','ElectricCruisePowerMargin_pct', ...
        'ThermalCruisePowerMargin_pct','PassesSeparateCruisePowerChecks'});

    writetable(SelectedDesign,'AE481_PS_Selected_Design.csv');
    disp(SelectedDesign);
end

fprintf('\nFiles written when this script completes:\n');
fprintf('  AE481_PS_Environmental.pdf (vector)\n');
fprintf('  AE481_PS_Environmental.png (600 dpi)\n');
fprintf('  AE481_PS_Engine_Candidates.csv\n');
fprintf('  AE481_PS_Selected_Design.csv (if a candidate passes)\n');

%% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function d = evaluatePSPoint(S_m2,P_kW,p)
    d.valid = false;
    d.S_m2 = S_m2;
    d.P_kW = P_kW;

    m = Weight_Predict_PS(S_m2,P_kW,p);
    if ~m.valid
        return;
    end

    W_N = m.mTO_kg*p.g0;
    WS_Nm2 = W_N/S_m2;
    PW_actual_WN = P_kW*1000/W_N;

    c = performanceConstraintsPS(WS_Nm2,m.CD0_clean,p);

    if ~isfinite(c.PW_threshold_WN)
        return;
    end

    P_takeoff_req_kW = c.PW_takeoff_WN*W_N/1000;
    P_roc_req_kW = c.PW_roc_WN*W_N/1000;
    P_initial_req_kW = c.PW_initial_WN*W_N/1000;
    P_clot_req_kW = c.PW_clot_WN*W_N/1000;
    P_balked_req_kW = c.PW_balked_WN*W_N/1000;
    P_cruise_req_kW = c.PW_cruise225_WN*W_N/1000;
    P_maneuver_req_kW = c.PW_maneuver_WN*W_N/1000;
    P_ceiling_req_kW = c.PW_ceiling_WN*W_N/1000;
    P_cruise250_req_kW = c.PW_cruise250_WN*W_N/1000;
    P_threshold_kW = c.PW_threshold_WN*W_N/1000;

    powerMarginFraction = P_kW/P_threshold_kW - 1;

    feasible = c.geometryOK && c.maneuverLiftOK ...
        && (P_kW >= P_threshold_kW) ...
        && (c.sLanding_m <= p.sLreq_m) ...
        && (m.mTO_kg <= p.MTOW_limit_kg);

    % Separate architecture diagnostics. The original model applies the
    % cruise power fraction to TOTAL hybrid power, so it does not prove
    % either power source can sustain a cruise segment on its own.
    % Use takeoff mass conservatively during electric cruise (no fuel burn).
    qCruise = 0.5*p.rho_cruise*p.Vcruise225_ms^2;
    CL_electric = W_N/(qCruise*S_m2);
    D_electric_N = qCruise*S_m2*(m.CD0_clean + ...
        CL_electric^2/(pi*p.AR*p.e_clean));
    P_electric_cruise_req_kW = D_electric_N*p.Vcruise225_ms/p.eta_cruise/1000;
    electricCruisePowerMargin = ...
        p.electricShare*P_kW/P_electric_cruise_req_kW - 1;
    % P_cruise_req_kW is installed-equivalent; undo its availability factor.
    P_thermal_cruise_req_kW = p.powerFrac_cruise*P_cruise_req_kW;
    thermalCruisePowerMargin = ...
        p.powerFrac_cruise*p.thermalShare*P_kW/P_thermal_cruise_req_kW - 1;

    % Operating climate proxy
    gridRecharge_kWh = m.EbatteryRequired_kWh/p.gridChargeEfficiency;
    operation_kgCO2 = p.fuelCO2_kg_per_kgFuel*m.mFuel_kg ...
        + p.gridCO2_kg_per_kWh*gridRecharge_kWh;

    % Manufacturing climate proxy
    manufacturing_kgCO2e = ...
        p.airframeManufacturing_kgCO2e_per_kgEmpty*m.mEmpty_kg ...
        + p.batteryManufacturing_kgCO2e_per_kWh*m.EpackNominal_kWh;

    objective = operation_kgCO2 + manufacturing_kgCO2e/p.lifeMissions;

    d.valid = true;
    d.mass = m;
    d.constraints = c;
    d.WS_Nm2 = WS_Nm2;
    d.PW_actual_WN = PW_actual_WN;
    d.powerMarginFraction = powerMarginFraction;
    d.feasible = feasible;
    d.electricCruisePowerMargin = electricCruisePowerMargin;
    d.thermalCruisePowerMargin = thermalCruisePowerMargin;

    d.margin_takeoff_kW = P_kW - P_takeoff_req_kW;
    d.margin_roc_kW = P_kW - P_roc_req_kW;
    d.margin_initial_kW = P_kW - P_initial_req_kW;
    d.margin_clot_kW = P_kW - P_clot_req_kW;
    d.margin_balked_kW = P_kW - P_balked_req_kW;
    d.margin_cruise225_kW = P_kW - P_cruise_req_kW;
    d.margin_maneuver_kW = P_kW - P_maneuver_req_kW;
    d.margin_ceiling_kW = P_kW - P_ceiling_req_kW;
    d.margin_cruise250_kW = P_kW - P_cruise250_req_kW;
    d.margin_landing_m = p.sLreq_m - c.sLanding_m;
    d.margin_mtow_kg = p.MTOW_limit_kg - m.mTO_kg;

    d.operation_kgCO2_perMission = operation_kgCO2;
    d.manufacturing_kgCO2e = manufacturing_kgCO2e;
    d.objective_kgCO2e_perMission = objective;
end

function c = performanceConstraintsPS(WS_Nm2,CD0clean,p)
    % Takeoff
    K_TO = 1/(pi*p.AR*p.e_TO);
    CL_ground = p.CLmax_TO/p.k_rotation^2;
    CD_ground = CD0clean + p.dCD0_TO_flaps + p.dCD0_gear ...
        + K_TO*CL_ground^2;

    [c.PW_takeoff_WN,c.geometryOK] = physicsTakeoffPW_PS( ...
        WS_Nm2,p.sTOreq_m,p.hObstacle_m,p.rho_field,p.CLmax_TO, ...
        CL_ground,CD_ground,p.mu_roll,p.eta_TO,p.powerFrac_TO, ...
        p.k_rotation,p.k_transition,p.n_transition,p.gamma_takeoff_deg);

    % RFP 1500 ft/min initial climb
    c.PW_roc_WN = climbPWfromROC_PS( ...
        WS_Nm2,p.beta_climb,p.rho_field,p.eta_climb, ...
        CD0clean+p.dCD0_TO_flaps+p.dCD0_gear,p.e_TO,p.AR, ...
        p.CLmax_TO,p.ks_initial,p.ROC_RFP_ms,1.0,p.powerFrac_climb);

    % 4% initial climb
    c.PW_initial_WN = climbPWfromGradient_PS( ...
        WS_Nm2,p.beta_climb,p.rho_field,p.eta_climb, ...
        CD0clean+p.dCD0_TO_flaps+p.dCD0_gear,p.e_TO,p.AR, ...
        p.CLmax_TO,p.ks_initial,p.G_initial,1.0,p.powerFrac_climb);

    % CLoT climb
    c.PW_clot_WN = climbPWfromGradient_PS( ...
        WS_Nm2,p.beta_climb,p.rho_field,p.eta_climb, ...
        CD0clean+p.dCD0_TO_flaps+p.dCD0_CLoT,p.e_CLoT,p.AR, ...
        p.CLmax_TO,p.ks_CLoT,p.G_CLoT,p.CLoT_power_remaining, ...
        p.powerFrac_climb);

    % Balked landing
    c.PW_balked_WN = climbPWfromGradient_PS( ...
        WS_Nm2,p.beta_landing,p.rho_field,p.eta_balked, ...
        CD0clean+p.dCD0_L_flaps+p.dCD0_gear,p.e_L,p.AR, ...
        p.CLmax_L,p.ks_balked,p.G_balked,1.0,p.powerFrac_balked);

    % Metabook Eq. 4.41 in SI form
    c.PW_cruise225_WN = cruisePW_PS( ...
        WS_Nm2,p.beta_cruise,p.rho_cruise,p.eta_cruise, ...
        CD0clean,p.e_clean,p.AR,p.Vcruise225_ms,p.powerFrac_cruise);

    c.PW_cruise250_WN = cruisePW_PS( ...
        WS_Nm2,p.beta_cruise,p.rho_cruise,p.eta_cruise, ...
        CD0clean,p.e_clean,p.AR,p.Vcruise250_ms,p.powerFrac_cruise);

    % Metabook maneuver equation
    [c.PW_maneuver_WN,c.maneuverLiftOK] = maneuverPW_PS( ...
        WS_Nm2,p.beta_maneuver,p.rho_cruise,p.eta_maneuver, ...
        CD0clean,p.e_clean,p.AR,p.CLmax_clean,p.V_maneuver_ms, ...
        p.n_maneuver,p.powerFrac_maneuver);

    % Service ceiling
    c.PW_ceiling_WN = serviceCeilingPW_PS( ...
        WS_Nm2,p.beta_ceiling,p.rho_ceiling,p.eta_ceiling, ...
        CD0clean,p.e_clean,p.AR,p.CLmax_clean,p.ks_ceiling, ...
        p.ROC_ceiling_ms,p.powerFrac_ceiling);

    c.sLanding_m = landingDistancePS(WS_Nm2,p);

    threshold = [ ...
        c.PW_takeoff_WN,c.PW_roc_WN,c.PW_initial_WN,c.PW_clot_WN, ...
        c.PW_balked_WN,c.PW_cruise225_WN,c.PW_maneuver_WN, ...
        c.PW_ceiling_WN];
    c.PW_threshold_WN = max(threshold);
end

function rho = atmDensityPS(h_m,dT_C)
    T0 = 288.15;
    p0 = 101325;
    L = 0.0065;
    g0 = 9.80665;
    R = 287.05287;
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
    TW_required = (kR^2.*WS_Nm2)./(g0*rho*CLmax.*sg_m) ...
        + mu + aeroGroundTerm;
    PWinstalled_WN = TW_required.*Vtr_ms./etaProp./powerAvailFrac;
    PWinstalled_WN(~geometryOK) = inf;
end

function PW = climbPWfromGradient_PS( ...
    WS_TO,betaW,rho,eta,CD0,e,AR,CLmax,ks,G,powerRemaining,powerAvailFrac)

    WS_local = betaW.*WS_TO;
    CL = CLmax/ks^2;
    V = sqrt(2.*WS_local./(rho*CL));
    K = 1/(pi*AR*e);
    CD = CD0 + K*CL^2;
    D_W = CD./CL;
    PW_current = V./eta.*(D_W + G);
    PW = betaW.*PW_current./(powerRemaining*powerAvailFrac);
end

function PW = climbPWfromROC_PS( ...
    WS_TO,betaW,rho,eta,CD0,e,AR,CLmax,ks,ROC,powerRemaining,powerAvailFrac)

    WS_local = betaW.*WS_TO;
    CL = CLmax/ks^2;
    V = sqrt(2.*WS_local./(rho*CL));
    G = ROC./V;
    PW = climbPWfromGradient_PS(WS_TO,betaW,rho,eta,CD0,e,AR, ...
        CLmax,ks,G,powerRemaining,powerAvailFrac);
end

function PW = cruisePW_PS(WS_TO,betaW,rho,eta,CD0,e,AR,V,powerAvailFrac)
    q = 0.5*rho*V^2;
    K = 1/(pi*AR*e);
    parasite = q*CD0./WS_TO;
    induced = K*betaW^2.*WS_TO./q;
    PW = V/(eta*powerAvailFrac).*(parasite + induced);
end

function [PW,liftOK] = maneuverPW_PS( ...
    WS_TO,betaW,rho,eta,CD0,e,AR,CLmax,V,n,powerAvailFrac)

    q = 0.5*rho*V^2;
    WS_m = betaW.*WS_TO;
    TW = q*CD0./WS_m + WS_m.*n^2./(q*pi*AR*e);
    PW = betaW.*TW.*V./(eta*powerAvailFrac);
    CL = n.*WS_m./q;
    liftOK = CL <= CLmax;
    PW(~liftOK) = inf;
end

function PW = serviceCeilingPW_PS( ...
    WS_TO,betaW,rho,eta,CD0,e,AR,CLmax,ksMin,ROC,powerAvailFrac)

    K = 1/(pi*AR*e);
    CLminP = sqrt(3.*CD0./K);
    CLstallMargin = CLmax/(ksMin^2);
    CL = min(CLminP,CLstallMargin);
    WS_local = betaW.*WS_TO;
    V = sqrt(2.*WS_local./(rho.*CL));
    CD = CD0 + K.*CL.^2;
    D_W = CD./CL;
    PW = betaW.*(V.*D_W + ROC)./(eta.*powerAvailFrac);
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
    if all(~isfinite(M(:)))
        return;
    end
    finiteM = M(isfinite(M));
    if isempty(finiteM) || min(finiteM) > 0 || max(finiteM) < 0
        return;
    end
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
