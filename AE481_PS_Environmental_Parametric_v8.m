%% AE481 - PARAMETRIC P-S CONSTRAINT DIAGRAM + ENVIRONMENTAL OBJECTIVE
% M-BRAER Hybrid-Electric STOL Commuter
%
% WORKSPACE & PIPELINE INTEGRATED VERSION

%% ========================================================================
% 1. LOAD WORKSPACE INPUTS & BASELINE FROM PIPELINE
% ========================================================================

if exist('Weight_Predict.m','file') ~= 2
    error(['Weight_Predict.m was not found. Rename the supplied ', ...
           'Weight_Predict file to Weight_Predict.m and place it here.']);
end
if exist('Weight_Predict_PS.m','file') ~= 2
    error('Weight_Predict_PS.m must be in the same MATLAB folder.');
end

% If run independently without the master runner, populate 'in' and baseline
if ~exist('in', 'var')
    fprintf('Running aircraft_inputs to populate design parameters...\n');
    aircraft_inputs;
end

if ~exist('values_kg', 'var')
    fprintf('Running Weight_Predict to generate weight vector...\n');
    values_kg = Weight_Predict(in);
end

% Inherit all parametric inputs directly from the centralized 'in' structure
p = in;

% Map baseline weights and reference design variables from the active workflow
p.mTO_baseline_kg      = values_kg(1);
p.mEmpty_baseline_kg   = values_kg(2);
p.mElectric_baseline_kg= values_kg(6);
p.mBattery_baseline_kg = values_kg(7);

if exist('S_design_m2', 'var') && exist('P_installed_kW', 'var') && exist('Swet_m2', 'var')
    p.S_ref_m2     = S_design_m2;
    p.P_ref_kW     = P_installed_kW;
    p.Swet_rest_m2 = (p.CD0_ref / p.Cf_equiv) * p.S_ref_m2 - 2 * p.S_ref_m2;
else
    % Fallback anchor if run standalone
    Wbase_N = p.mTO_baseline_kg * p.g0;
    WS_ref = 690.0;
    p.S_ref_m2 = Wbase_N / WS_ref;
    p.Swet_rest_m2 = (p.CD0_ref / p.Cf_equiv) * p.S_ref_m2 - 2 * p.S_ref_m2;
    p.P_ref_kW = 1812.2; % Default baseline power estimate
end

mTO_baseline_kg       = p.mTO_baseline_kg;
mEmpty_baseline_kg    = p.mEmpty_baseline_kg;
mElectric_baseline_kg = p.mElectric_baseline_kg;

% Populate derived operational velocities and densities
p.Vcruise225_ms = p.cruise_kt * 0.514444;
p.Vcruise250_ms = 250 * 0.514444; 
p.sTOreq_m      = p.s_TO_req_m;
p.sLreq_m       = p.s_L_req_m;
p.hObstacle_m   = p.h_obstacle_m;
p.ROC_RFP_ms    = p.ROC_RFP_ms;

p.rho_field   = atmDensityPS(p.h_field_m, p.dT_field_C);
p.rho_cruise  = atmDensityPS(p.h_cruise_m, p.dT_cruise_C);
p.rho_ceiling = atmDensityPS(p.h_ceiling_m, p.dT_ceiling_C);

%% ========================================================================
% 2. ENVIRONMENTAL OBJECTIVE - SIMPLE CLIMATE PROXY
% ========================================================================
% Inherited directly from centralized pipeline (in.* fields mapped to p.*)

%% ========================================================================
% 3. BUILD A BASELINE (Sref, Pref) - ANCHORED TO CONSTRAINT DIAGRAM
% ========================================================================

Wbase_N = mTO_baseline_kg * p.g0;

WS_ref = Wbase_N / p.S_ref_m2;
S_ref_m2 = p.S_ref_m2;
P_ref_kW = p.P_ref_kW;

fprintf('\n============================================================\n');
fprintf('ANCHORED BASELINE FOR P-S SWEEP\n');
fprintf('============================================================\n');
fprintf('Baseline mass        : %.1f kg\n', mTO_baseline_kg);
fprintf('Design W/S           : %.1f N/m^2\n', WS_ref);
fprintf('Reference S          : %.2f m^2\n', S_ref_m2);
fprintf('Reference P          : %.1f kW\n', P_ref_kW);

%% ========================================================================
% 4. P-S SWEEP
% ========================================================================

engineName = {'GE Catalyst (1300 shp)'; 'Honeywell TPE331-14GR'; ...
    'Pratt & Whitney PT6A-67F'; 'GE CT7-9B (normal takeoff)'};
engineSHP = [1300; 1650; 1700; 1750];
engineRated_kW = engineSHP*0.745699872;

engineNetThermal_kW = engineRated_kW*(1-p.engineOfftakeFraction);
engineSystemPower_kW = engineNetThermal_kW/p.thermalShare;
engineElectric_kW = p.electricShare*engineSystemPower_kW;

plotPowerScale = 1;

nS = 81;
nP = 91;
S_vec_m2 = linspace(0.80*S_ref_m2,1.48*S_ref_m2,nS);
P_vec_kW = linspace(min(0.68*P_ref_kW,0.95*min(engineSystemPower_kW)), ...
    max(1.28*P_ref_kW,1.05*max(engineSystemPower_kW)),nP);
P_lower_kW = linspace(1,min(P_vec_kW),61);
P_vec_kW = [P_lower_kW(1:end-1),P_vec_kW];
nP = numel(P_vec_kW);
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
M_cruise250 = nan(sz);

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
% 5. EXISTING ENGINE LINES + WIGGLE ROOM
% ========================================================================

designPowerMargin = 0.01;

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
                bestD = d;
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
        engineElectricCruiseMargin_pct(iE) = 100*bestD.electricCruisePowerMargin;
        engineThermalCruiseMargin_pct(iE) = 100*bestD.thermalCruisePowerMargin;
        engineBestS_m2(iE) = bestD.S_m2;
        engineBestObjective(iE) = bestD.objective_kgCO2e_perMission;
        engineBestPowerMargin(iE) = bestD.powerMarginFraction;
        engineBestLanding_m(iE) = bestD.constraints.sLanding_m;
        engineBestMTO_kg(iE) = bestD.mass.mTO_kg;

        if bestJ < selectedObjective
            selectedObjective = bestJ;
            selectedDesign = bestD;
            selectedEngineIndex = iE;
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
% 6. PLOT: FEASIBLE REGION + CONSTRAINTS + ENVIRONMENTAL CONTOURS
% ========================================================================

P_plot_grid = plotPowerScale*P_grid_kW;

figure('Color','w','Position',[70 70 1350 820]);
hold on; grid on; box on;

if any(Feasible(:))
    [~,hFeas] = contourf(S_grid_m2,P_plot_grid,double(Feasible),[0.5 1.5], ...
        'LineStyle','none');
    hFeas.FaceAlpha = 0.18;
    hFeas.DisplayName = 'Feasible under P-S model';
end

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

cc = lines(10);
plotZeroContour(S_grid_m2,P_plot_grid,M_takeoff,'Takeoff <= 300 ft',1.6,cc(1,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_roc, sprintf('ROC >= %d ft/min', in.climb_ftmin), 1.6,cc(2,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_initial,'Initial climb >= 4%',1.6,cc(3,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_clot,'CLoT climb >= 1%',1.6,cc(4,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_balked,'Balked landing >= 3%',1.6,cc(5,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_cruise,'Cruise >= 225 kt',2.0,cc(6,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_maneuver,sprintf('Sustained %d deg bank', in.phi_maneuver_deg),2.0,cc(7,:));
plotZeroContour(S_grid_m2, P_plot_grid, M_ceiling, sprintf('Service ceiling %d ft', in.serviceCeiling), 1.6, cc(8,:));
plotZeroContour(S_grid_m2,P_plot_grid,M_landing,'Landing <= 300 ft',2.0,[1 0.5 0]);
plotZeroContour(S_grid_m2,P_plot_grid,M_mtow,'Part 23 MTOW <= 19,000 lb',1.6,cc(10,:));

[C250,h250] = contour(S_grid_m2,P_plot_grid,M_cruise250,[0 0], ...
    ':','LineWidth',1.5,'LineColor',[0.35 0.35 0.35]);
h250.DisplayName = '250-kt cruise objective';

xlabel('Wing Reference Area, S [m^2]');
ylabel('Total Power [kW]');
title({'M-BRAER Parametric P-S Constraint Diagram', ...
       'Environmental objective contours = kg CO_2e per mission equivalent'});

xlim([min(S_vec_m2) max(S_vec_m2)]);
ylim(plotPowerScale*[min(P_vec_kW) max(P_vec_kW)]);

% Find the feasible point that minimizes CO2 emissions in the P-S sweep
validFeas = Feasible & isfinite(J_kgCO2eMission);

if any(validFeas(:))
    [~, idxOpt] = min(J_kgCO2eMission(validFeas));
    feasS = S_grid_m2(validFeas);
    feasP = P_grid_kW(validFeas);
    optS = feasS(idxOpt);
    optP = feasP(idxOpt);
else
    optS = S_ref_m2;
    optP = P_ref_kW;
end

plot(optS, plotPowerScale * optP, 'kp', ...
    'MarkerSize', 16, 'MarkerFaceColor', 'y', 'LineStyle', 'none', ...
    'DisplayName', sprintf('Optimal P-S Design (%.2f m^2, %.1f kW)', optS, optP));

legend('Location','east');
exportFigurePS('AE481_PS_Environmental');

%% ========================================================================
% 7. SELECTED DESIGN REPORT
% ========================================================================

fprintf('\n============================================================\n');
fprintf('DISCRETE ENGINE CANDIDATES\n');
fprintf('============================================================\n');
disp(EngineCandidates);

% Evaluate the exact continuous grid-minimum point found for the plot star
selectedDesign = evaluatePSPoint(optS, optP, p);

if ~selectedDesign.valid
    warning('The optimal P-S grid point could not be fully evaluated.');
else
    fprintf('\n============================================================\n');
    fprintf('SELECTED P-S DESIGN (GLOBAL OPTIMAL FROM PLOT)\n');
    fprintf('============================================================\n');
    fprintf('Wing area S             : %.2f m^2\n', selectedDesign.S_m2);
    fprintf('Total Installed Power   : %.1f kW\n', selectedDesign.P_kW);
    fprintf('Converged MTOW          : %.1f kg (%.0f lb)\n', ...
        selectedDesign.mass.mTO_kg, selectedDesign.mass.mTO_kg / 0.45359237);
    fprintf('W/S                     : %.1f N/m^2\n', selectedDesign.WS_Nm2);
    fprintf('Required P/W            : %.2f W/N\n', ...
        selectedDesign.constraints.PW_threshold_WN);
    fprintf('Available P/W           : %.2f W/N\n', selectedDesign.PW_actual_WN);
    fprintf('Installed-power margin  : %.1f%%\n', ...
        100 * selectedDesign.powerMarginFraction);
    fprintf('Landing distance        : %.2f m\n', selectedDesign.constraints.sLanding_m);
    fprintf('Mission CO2e Objective  : %.2f kg CO2e\n', selectedDesign.objective_kgCO2e_perMission);

    SelectedDesign = table( ...
        selectedDesign.S_m2, selectedDesign.P_kW, ...
        selectedDesign.mass.mTO_kg, selectedDesign.WS_Nm2, ...
        selectedDesign.PW_actual_WN, selectedDesign.constraints.PW_threshold_WN, ...
        100 * selectedDesign.powerMarginFraction, ...
        selectedDesign.constraints.sLanding_m, ...
        selectedDesign.mass.mFuel_kg, selectedDesign.mass.mBattery_kg, ...
        selectedDesign.mass.EpackNominal_kWh, selectedDesign.mass.LD_cruise, ...
        selectedDesign.operation_kgCO2_perMission, ...
        selectedDesign.manufacturing_kgCO2e, ...
        selectedDesign.objective_kgCO2e_perMission, ...
        'VariableNames', { ...
        'WingArea_m2', 'InstalledPower_kW', 'MTOW_kg', 'WS_Nm2', ...
        'PWavailable_WN', 'PWrequired_WN', 'PowerMargin_pct', 'LandingDistance_m', ...
        'Fuel_kg', 'Battery_kg', 'BatteryNominal_kWh', 'Cruise_LD', ...
        'OperatingCO2_kgMission', 'ManufacturingCO2e_kg', ...
        'ObjectiveCO2e_kgMission'});

    writetable(SelectedDesign, 'AE481_PS_Selected_Design.csv');
    disp(SelectedDesign);
end

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

    qCruise = 0.5*p.rho_cruise*p.Vcruise225_ms^2;
    CL_electric = W_N/(qCruise*S_m2);
    D_electric_N = qCruise*S_m2*(m.CD0_clean + ...
        CL_electric^2/(pi*p.AR*p.e_clean));
    P_electric_cruise_req_kW = D_electric_N*p.Vcruise225_ms/p.eta_cruise/1000;
    electricCruisePowerMargin = ...
        p.electricShare*P_kW/P_electric_cruise_req_kW - 1;
    P_thermal_cruise_req_kW = p.powerFrac_cruise*P_cruise_req_kW;
    thermalCruisePowerMargin = ...
        p.powerFrac_cruise*p.thermalShare*P_kW/P_thermal_cruise_req_kW - 1;

    gridRecharge_kWh = m.EbatteryRequired_kWh/p.gridChargeEfficiency;
    operation_kgCO2 = p.fuelCO2_kg_per_kgFuel*m.mFuel_kg ...
        + p.gridCO2_kg_per_kWh*gridRecharge_kWh;

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
    K_TO = 1/(pi*p.AR*p.e_TO);
    CL_ground = p.CLmax_TO/p.k_rotation^2;
    CD_ground = CD0clean + p.dCD0_TO_flaps + p.dCD0_gear ...
        + K_TO*CL_ground^2;

    [c.PW_takeoff_WN,c.geometryOK] = physicsTakeoffPW_PS( ...
        WS_Nm2,p.sTOreq_m,p.hObstacle_m,p.rho_field,p.CLmax_TO, ...
        CL_ground,CD_ground,p.mu_roll,p.eta_TO,p.powerFrac_TO, ...
        p.k_rotation,p.k_transition,p.n_transition,p.gamma_takeoff_deg);

    c.PW_roc_WN = climbPWfromROC_PS( ...
        WS_Nm2,p.beta_climb,p.rho_field,p.eta_climb, ...
        CD0clean+p.dCD0_TO_flaps+p.dCD0_gear,p.e_TO,p.AR, ...
        p.CLmax_TO,p.ks_initial,p.ROC_RFP_ms,1.0,p.powerFrac_climb);

    c.PW_initial_WN = climbPWfromGradient_PS( ...
        WS_Nm2,p.beta_climb,p.rho_field,p.eta_climb, ...
        CD0clean+p.dCD0_TO_flaps+p.dCD0_gear,p.e_TO,p.AR, ...
        p.CLmax_TO,p.ks_initial,p.G_initial,1.0,p.powerFrac_climb);

    c.PW_clot_WN = climbPWfromGradient_PS( ...
        WS_Nm2,p.beta_climb,p.rho_field,p.eta_climb, ...
        CD0clean+p.dCD0_TO_flaps+p.dCD0_CLoT,p.e_CLoT,p.AR, ...
        p.CLmax_TO,p.ks_CLoT,p.G_CLoT,p.CLoT_power_remaining, ...
        p.powerFrac_climb);

    c.PW_balked_WN = climbPWfromGradient_PS( ...
        WS_Nm2,p.beta_landing,p.rho_field,p.eta_balked, ...
        CD0clean+p.dCD0_L_flaps+p.dCD0_gear,p.e_L,p.AR, ...
        p.CLmax_L,p.ks_balked,p.G_balked,1.0,p.powerFrac_balked);

    c.PW_cruise225_WN = cruisePW_PS( ...
        WS_Nm2,p.beta_cruise,p.rho_cruise,p.eta_cruise, ...
        CD0clean,p.e_clean,p.AR,p.Vcruise225_ms,p.powerFrac_cruise);

    c.PW_cruise250_WN = cruisePW_PS( ...
        WS_Nm2,p.beta_cruise,p.rho_cruise,p.eta_cruise, ...
        CD0clean,p.e_clean,p.AR,p.Vcruise250_ms,p.powerFrac_cruise);

    [c.PW_maneuver_WN,c.maneuverLiftOK] = maneuverPW_PS( ...
        WS_Nm2,p.beta_maneuver,p.rho_cruise,p.eta_maneuver, ...
        CD0clean,p.e_clean,p.AR,p.CLmax_clean,p.V_maneuver_ms, ...
        p.n_maneuver,p.powerFrac_maneuver);

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
    if any(above)
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
    if any(above)
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