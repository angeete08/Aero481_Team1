function [prop, args, src] = AE481_Propulsion_CT7_Pipeline(in, design, varargin)
%AE481_PROPULSION_CT7_PIPELINE  Run the CT7 propulsion analysis at the pipeline's design point.
%
%   prop = AE481_Propulsion_CT7_Pipeline(in, selectedDesign)
%   prop = AE481_Propulsion_CT7_Pipeline(in, selectedDesign, 'P_aux', 30, ...)   extra CT7 inputs by name
%   [prop, args, src] = ...   also returns the name-value list passed to AE481_Propulsion_CT7 and its sources
%
% Called from run_aircraft_design.m (step 12) after the final P-S sweep. Analysis only: nothing it returns
% is written back into the weights, wing area or power of the team pipeline.
%
% Inputs
%   in              the centralised input structure from aircraft_inputs.m
%   selectedDesign  the final P-S design from AE481_PS_Environmental_Parametric_v8.m (evaluatePSPoint output:
%                   .S_m2, .P_kW, .mass (Weight_Predict_PS result), .constraints (performanceConstraintsPS))
%
% Which value wins where the two codes overlap (agreed 20/11/2026):
%   - Aircraft, design point and mission numbers come from the team code (aircraft_inputs.m and the P-S design),
%     including the efficiencies both codes model (propeller, battery, motor, inverter, cable).
%   - Propulsion-only numbers stay as set in AE481_Propulsion_CT7.m: the CT7-8A6 engine, propellers, blowers,
%     non-propulsive electrical load, offtakes, gearbox, connectors and TMS.
%   - The M/G rating is the P-S cruise constraint power at the design point (as in the PDR); cruise power
%     itself is computed from the drag, so the printout shows how much of the rating cruise uses.
%   - The mission schedule stays the CT7 one: E1 + fuel leg (with recharge) + E2, each electric leg
%     in.electricRange_nmi long, climb and descent flown on top of in.range_nmi of cruise.
%
% Kit Man Chan, 20/11/2026. Plain MATLAB (R2016b or newer), no toolboxes.

if ~isstruct(design) || ~isfield(design, 'valid') || ~design.valid
    error(['AE481_Propulsion_CT7_Pipeline: the P-S sweep returned no valid design point, ', ...
           'so there is nothing to run the propulsion analysis at.']);
end
g0 = 9.80665;
m  = design.mass;                   % Weight_Predict_PS result at the selected (S, P)
c  = design.constraints;            % P/W of each constraint at the selected W/S [W/N]
W_kN = m.mTO_kg*g0/1000;            % take-off weight [kN], so P/W [W/N] x W [kN] = P [kW]

% Shaft power each P-S constraint needs at the propellers [kW]. The team's P/W values are installed power,
% i.e. shaft power / powerFrac (and / CLoT_power_remaining for the climb after engine failure), so multiply back.
PS_need = W_kN*[ ...
    c.PW_takeoff_WN    *in.powerFrac_TO, ...
    c.PW_roc_WN        *in.powerFrac_climb, ...
    c.PW_initial_WN    *in.powerFrac_climb, ...
    c.PW_clot_WN       *in.powerFrac_climb*in.CLoT_power_remaining, ...
    c.PW_balked_WN     *in.powerFrac_balked, ...
    c.PW_cruise225_WN  *in.powerFrac_cruise, ...
    c.PW_maneuver_WN   *in.powerFrac_maneuver, ...
    c.PW_ceiling_WN    *in.powerFrac_ceiling, ...
    c.PW_cruise250_WN  *in.powerFrac_cruise];
P_cruise_PS = c.PW_cruise225_WN*W_kN;   % P-S cruise constraint, installed power [kW]: the M/G rating

% name, value, source -- one row per CT7 input set from the pipeline
map = {
    % design point (final P-S sweep)
    'm_TO',          m.mTO_kg,                 'P-S design: MTOW';
    'S',             design.S_m2,              'P-S design: wing area';
    'CD0',           m.CD0_clean,              'P-S design: clean CD0';
    'mB_fix',        m.mBattery_kg,            'P-S design: battery';
    'm_fuel_tank',   m.mFuel_kg,               'P-S design: fuel incl. reserve';
    'P_MG_fixed',    P_cruise_PS,              'P-S design: 225 kt cruise constraint (installed)';
    'PS_need',       PS_need,                  'P-S design: constraint powers x powerFrac';
    % aircraft
    'AR',            in.AR,                    'in.AR';
    'e',             in.e_clean,               'in.e_clean';
    'beta_cr',       in.beta_cruise,           'in.beta_cruise';
    % mission
    'V_kt',          in.cruise_kt,             'in.cruise_kt';
    'h_cr_ft',       in.altitude_ft,           'in.altitude_ft';
    'R_des',         in.range_nmi,             'in.range_nmi';
    'R_E',           in.electricRange_nmi,     'in.electricRange_nmi (each electric leg)';
    'V_clb_kt',      in.climb_kt,              'in.climb_kt';
    'ROC_clb_ftmin', in.climb_ftmin,           'in.climb_ftmin';
    'V_dsc_kt',      in.descent_kt,            'in.descent_kt';
    'ROD_dsc_ftmin', in.descent_ftmin,         'in.descent_ftmin';
    't_res',         in.reserve_min,           'in.reserve_min';
    'R_alt',         in.alternate_nmi,         'in.alternate_nmi';
    'R_emerg_req',   in.engineOut_nmi,         'in.engineOut_nmi (battery-only diversion)';
    'cp_lb',         in.Cp,                    'in.Cp';
    % efficiencies both codes model
    'eta_p',         in.eta_cruise,            'in.eta_cruise';
    'eta_clb',       in.eta_climb,             'in.eta_climb';
    'eta_batt',      in.etaBattery,            'in.etaBattery';
    'eta_mot',       in.etaMotor,              'in.etaMotor';
    'eta_inv',       in.etaInverter,           'in.etaInverter';
    'eta_wire',      in.etaCable,              'in.etaCable';
    % battery and fuel system
    'e_b',           in.pack_Whkg,             'in.pack_Whkg';
    'p_b',           in.pack_kWkg,             'in.pack_kWkg';
    'SOC_min',       1 - in.usableSOC,         '1 - in.usableSOC';
    'm_trap',        in.trappedFuel_kg,        'in.trappedFuel_kg';
    'n_tank',        in.fuelTankCount,         'in.fuelTankCount';
    'k_pg',          in.kPropulsionGroup,      'in.kPropulsionGroup';
    };
args = reshape(map(:, 1:2)', 1, []);     % name-value list
args = [args, varargin];                 % caller's own changes last, so they win
src  = map(:, [1 3]);

% Print what was passed and where it came from
fprintf('\n------------------------------------------------------------------------------------------\n');
fprintf('CT7 PROPULSION ANALYSIS AT THE FINAL P-S DESIGN POINT (analysis only, nothing fed back)\n');
fprintf('------------------------------------------------------------------------------------------\n');
fprintf('Inputs taken from the pipeline (everything else: AE481_Propulsion_CT7.m defaults)\n');
for k = 1:size(map, 1)
    v = map{k, 2};
    if numel(v) == 1
        fprintf('   %-14s %12.5g   %s\n', map{k, 1}, v, map{k, 3});
    else
        fprintf('   %-14s %s\n                               %s\n', map{k, 1}, mat2str(round(v*10)/10), map{k, 3});
    end
end
if abs(in.alternate_nmi - in.reserve_kt*in.reserve_min/60) < 1e-6
    fprintf(['   NOTE: in.alternate_nmi (%.2f nmi) is the %g-min reserve distance, and the %g-min reserve is ', ...
             'added on top,\n         so the reserve fuel printed below covers the reserve twice.\n'], ...
            in.alternate_nmi, in.reserve_min, in.reserve_min);
end

prop = AE481_Propulsion_CT7(args{:}, 'print', true);

% Short summary of the numbers the pipeline usually needs
fprintf('\nCT7 SUMMARY\n');
fprintf('   M/G rating %.0f kW; electric cruise uses %.0f kW (%.0f%% of it)\n', ...
    prop.P_MG, prop.P_sh_cr, 100*prop.P_sh_cr/prop.P_MG);
fprintf('   Battery %.0f kg (%.0f kWh): lowest charge %.1f%%, at landing %.1f%% (floor %.0f%%)\n', ...
    prop.mB, prop.E_nom, 100*min(prop.soc), 100*prop.soc(end), 100*prop.inputs.SOC_min);
fprintf('   Battery that would just close the mission: %.0f kg (energy) / %.0f kg (power)\n', prop.mB_E, prop.mB_P);
fprintf('   Fuel %.0f kg (trip %.0f + reserve %.0f) vs %.0f kg carried by the P-S design\n', ...
    prop.m_fuel, prop.m_block, prop.m_res45, m.mFuel_kg);
fprintf('   Smallest power margin over the P-S constraints: %+.1f%% normal, %+.1f%% icing (250 kt objective excluded)\n', ...
    100*min(prop.chk_mar(1:end-1)), 100*min(prop.chk_mari(1:end-1)));
fprintf('   Propulsion hardware (itemised) %.1f kg, %.1f kg without the battery\n', prop.m_items, prop.m_items - prop.mB);
end
