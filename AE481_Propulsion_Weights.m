function r = AE481_Propulsion_Weights(varargin)
%AE481_PROPULSION_WEIGHTS  Stand-alone propulsion weight calculation for M-BRAER
%   (GE CT7-8A6 turboshaft + cruise motor/generator (M/G) + 8 x 150 kW blown-lift propulsors + battery).
%
%   AE481_Propulsion_Weights                  prints the weight breakdown (same as pressing Run)
%   r = AE481_Propulsion_Weights;             returns every number in struct r, nothing printed
%   r = AE481_Propulsion_Weights('J', 5e6);   change any input of weightInputs by name (here: 5 A/mm^2 cables)
%   r = AE481_Propulsion_Weights('print', true, ...)   print and return the struct
%
% What it does
%   Only the weights. It is split out of AE481_Propulsion_CT7.m (the master propulsion code) and uses the
%   same equations and input names, so both give the same masses for the same inputs. It needs just two
%   power numbers from the rest of the propulsion analysis, and works them out itself in section 2:
%     - the M/G rating, from the 225 kt cruise drag (the M/G is sized so that cruise is 80% of its rating);
%     - the heat the thermal-management system (TMS) must reject in electric cruise.
%   The battery mass is an INPUT here (1,050 kg, the PDR pack). Sizing the battery needs the whole mission
%   energy analysis, which stays in AE481_Propulsion_CT7.m.
%
% These are the real component weights: product data where a product is selected, otherwise a sourced
% equation or a stated estimate. The basis of each item is printed next to it. Nothing here is taken from
% the A7 weight table (Table 2), whose propulsion rows were set for the assignment.
%
% What it does not do
%   No feedback to MTOW: the masses do not change the MTOW, wing area or drag used in section 2. Closing
%   MTOW is the job of the team's P-S / weight code. Copied inputs are tagged so they can be updated by hand.
%
% Kit Man Chan, 09/11/2026. Plain MATLAB (R2016b or newer), no toolboxes.
%
% Input sources (in square brackets after each input):
%   [TCDS] EASA TCDS IM.E.010, GE CT7 series, Issue 10 (19/12/2025): ratings, dry mass
%   [H3X]  H3X HPDM-1500 (125 kg) and HPDM-180D (13.7 kg) integrated motor drives, inverter included
%   [R408] EASA TCDS P.002 Issue 3: Dowty R408, 6 composite blades, 4.115 m, 3,782 kW, 252 kg with spinner
%   [HZ]   Hartzell Carbon Voyager: three-blade carbon propeller, 54 lb (24.5 kg) (FLYING, 13/01/2025)
%   [RAY]  Raymer, Aircraft Design, 4th ed., Ch. 15: fuel-system, engine-control and starter equations
%   [MB]   Martins, AE481 metabook, Eq. (7.21): propulsion-group mass
%   [NASA] Woodworth, NASA EAP keynote (2023): 13 kW/kg motors, 19 kW/kg and 99% inverters
%   [PS]   Team P-S code v8, commit 8cd6c3b: design point (copied by hand)
%   [IN]   Team input file aircraft_inputs.m, commit 8cd6c3b (copied by hand)
%   [A7]   Team A7 inputs: fuel load and component stations (copied by hand)
%   [TEAM] Team technology set already used in the propulsion section
%   [EST]  Our engineering estimate

%% 1. Inputs: defaults from weightInputs (bottom of this file), then any name-value changes
g0 = 9.80665;          % standard gravity [m/s^2]
KT = 0.514444;         % 1 kt in m/s
FT = 0.3048;           % 1 ft in m
LB = 0.45359237;       % 1 lb in kg
HP = 0.745699872;      % 1 hp in kW
if mod(numel(varargin), 2) ~= 0
    error('AE481_Propulsion_Weights: give inputs as name-value pairs, for example (''J'', 5e6)');
end
doPrint = (nargout == 0);               % print when called without an output (e.g. pressing Run)
for k = 1:2:numel(varargin)
    if strcmp(varargin{k}, 'print')
        doPrint = varargin{k+1};
    end
end
p = weightInputs();                     % default inputs
for k = 1:2:numel(varargin)             % apply the changes the caller asked for
    name = varargin{k};
    if ~strcmp(name, 'print')
        if ~isfield(p, name)
            error('AE481_Propulsion_Weights: unknown input ''%s'' (see weightInputs)', name);
        end
        p.(name) = varargin{k+1};
    end
end

%% 2. The two power numbers the weights need
% (a) M/G rating from the 225 kt cruise drag at the P-S design point
V      = p.V_kt*KT;                      % cruise speed [m/s]
h_cr   = p.h_cr_ft*FT;                   % cruise altitude [m]
rho_cr = atmISA(h_cr, 0);                % air density at cruise altitude, ISA [kg/m^3]
K      = 1/(pi*p.AR*p.e);                % induced-drag factor [-]
q_cr   = 0.5*rho_cr*V^2;                 % dynamic pressure [Pa]
W_cr   = p.beta_cr*p.m_TO*g0;            % cruise weight [N]
CL_cr  = W_cr/(q_cr*p.S);                % cruise lift coefficient [-]
LD_cr  = CL_cr/(p.CD0 + K*CL_cr^2);      % cruise lift-to-drag ratio, clean airframe [-]
if isempty(p.P_MG_fixed)
    P_sh_cr0 = W_cr/LD_cr*V/p.eta_p/1000;    % propeller shaft power for 225 kt, clean airframe [kW]
    P_MG     = P_sh_cr0/p.f_cr;              % M/G rating: cruise runs at f_cr of it [kW]
else
    P_MG     = p.P_MG_fixed;                 % rating given directly (e.g. 1,334 kW in the PDR) [kW]
    P_sh_cr0 = p.f_cr*P_MG;                  % cruise shaft power that rating implies [kW]
end
P_sh_cr = P_sh_cr0*(1 + p.dD_ram);       % electric-leg shaft power, with the TMS cooling-air drag [kW]

% (b) Efficiency chain from the battery terminals to the propeller
t2s = p.eta_wire*p.eta_conn*p.eta_inv*p.eta_tms*p.eta_mot;   % terminals -> motor shaft [-]
t2p = t2s*p.eta_gb;                                           % terminals -> propeller shaft [-]

% (c) Heat the TMS rejects in electric cruise (the sizing case): every loss between the cells and the propeller
P_ecr      = P_sh_cr/t2p + p.P_aux;      % power at the battery terminals, incl. the non-propulsive load [kW]
P_cells_cr = P_ecr/p.eta_batt;           % power drawn from the cells [kW]
P_aw       = (P_ecr - p.P_aux)*p.eta_wire*p.eta_conn;   % propulsive power reaching the inverter [kW]
Q_batt = P_cells_cr - P_ecr;                                  % battery loss [kW]
Q_inv  = P_aw*(1 - p.eta_inv);                                % inverter loss [kW]
Q_mot  = P_aw*p.eta_inv*p.eta_tms*(1 - p.eta_mot);            % motor loss [kW]
Q_gb   = P_aw*p.eta_inv*p.eta_tms*p.eta_mot*(1 - p.eta_gb);   % gearbox loss [kW]
Q      = Q_batt + Q_inv + Q_mot + Q_gb;                       % total heat to reject [kW]

% (d) Battery power capability (sets the pack cable current)
E_nom  = p.mB*p.e_b/1000;                % pack energy [kWh]
P_bmax = p.mB*p.p_b;                     % pack power capability [kW]

%% 3. Engine side
% Eq. 7.21 of the metabook: propulsion-group mass for a turboprop installation (engine, propeller, fuel and
% oil systems, controls, installation), from the dry engine mass and take-off power, in pounds and hp
m_pg = p.k_pg*(p.m_eng/LB + 0.24*p.P_TO/HP)*LB;               % [kg] [MB]
% Combining gearbox: 0.03 kg/kW of engine + M/G power for the gearbox and wet clutch, plus 0.02 kg/kW of
% engine power for the extra high-speed stage that takes the 21,945 rpm engine shaft
m_gb = p.gb_comb*(p.P_TO + P_MG) + p.gb_hs*p.P_TO;           % [kg] [TEAM/EST]
% Main propeller: the Dowty R408 (same blade count) scaled by disc area to our diameter
m_mainprop = p.m_R408*(p.D_prop/p.D_R408)^2*(p.n_blade/6);    % [kg] [R408]
% Trapped (unusable) fuel: a fraction of the fuel load
m_trap = p.f_trap*p.m_fuel_tank;                              % [kg] [EST]
% Fuel system, Raymer general-aviation equation for integral tanks (V_t total, V_i integral = V_t)
Vt_gal = (p.m_fuel_tank + m_trap)/0.8/3.785411784;            % tank volume at 0.8 kg/L [US gal]
m_fs   = 2.49*Vt_gal^0.726*(1/(1 + 1))^0.363*p.n_tank^0.242*1^0.157*LB;   % [kg] [RAY], 1 engine
% Engine controls and starter, Raymer transport equations, one engine
W_en_lb = p.m_eng/LB;                                         % dry engine weight [lb]
m_ec = (5.0*1 + 0.80*p.L_ec/FT)*LB;                           % controls: 5 lb per engine + 0.8 lb/ft of run [kg]
m_st = 49.19*(1*W_en_lb/1000)^0.541*LB;                       % starter [kg]

%% 4. Electric side
% (a) Parametric masses at NASA technology goals (as in the PDR): motors at sp_mot kW/kg, inverters at
%     sp_inv kW/kg of electrical input, plus an installation fraction
m_MG    = P_MG/p.sp_mot;                 % M/G motor [kg]
m_iMG   = P_MG/p.eta_mot/p.sp_inv;       % M/G inverter [kg]
m_EPm   = p.P_EP/p.sp_mot;               % one blower motor [kg]
m_iEP   = p.P_EP/p.eta_mot/p.sp_inv;     % one blower inverter [kg]
m_mot   = m_MG + p.n_EP*m_EPm;           % all motors [kg]
m_inv   = m_iMG + p.n_EP*m_iEP;          % all inverters [kg]
m_inst  = p.f_inst*(m_mot + m_inv);      % mounts, cooling plates, local wiring [kg]
m_props = p.n_EP*p.m_EPprop;             % blower propellers [kg]
m_1EP   = (m_EPm + m_iEP)*(1 + p.f_inst) + p.m_EPprop;   % one complete blown-lift propulsor [kg]
% (b) Selected products (itemised list): integrated motor drives, inverter inside the motor
m_MGunit_tot = p.m_MGunit;               % M/G drive [kg] [H3X]
m_EPunit_tot = p.n_EP*p.m_EPunit;        % all blower drives [kg] [H3X]

% (c) High-voltage DC harness, aluminium. Each run: n runs x nc conductors x routed length x area x density,
%     area = current / current density, plus insulation (f_ins); connectors and protection (f_cn) at the end.
cableMass = @(I, L, n, nc) n*nc*p.k_route*L*(I/p.J)*p.rho_Al*(1 + p.f_ins);   % [kg]
I_sec = P_bmax/p.n_sec/p.V_bus*1e3;                  % current from each pack section at full pack power [A]
I_MG  = P_MG/p.eta_mot/p.eta_inv/p.V_bus*1e3;        % M/G feeder current at full rating [A]
I_EP  = p.P_EP/p.eta_mot/p.eta_inv/p.V_bus*1e3;      % each blower feeder at full power [A]
m_cpack = cableMass(I_sec, p.L_pack, p.n_sec, 2);    % pack sections to the bus, + and - conductors [kg]
m_cMG   = cableMass(I_MG, p.L_MGc, 1, 2);            % bus to the M/G [kg]
m_cEP   = 0;                                         % bus to the blowers, both wings [kg]
for y = p.y_EP                                       % one run per blower station, each side (2 runs)
    m_cEP = m_cEP + cableMass(I_EP, y, 2, 2);
end
m_cAC = 0;                                           % AC phase cables (only with separate inverters) [kg]
if p.ac_links
    I_acMG = P_MG/p.eta_mot/(sqrt(3)*p.V_ac*p.pf)*1e3;     % M/G phase current [A]
    I_acEP = p.P_EP/p.eta_mot/(sqrt(3)*p.V_ac*p.pf)*1e3;   % blower phase current [A]
    m_cAC  = cableMass(I_acMG, p.L_ac/p.k_route, 1, 3) + cableMass(I_acEP, p.L_ac/p.k_route, p.n_EP, 3);
end
m_c    = m_cpack + m_cMG + m_cEP + m_cAC;            % conductors + insulation [kg]
m_wire = m_c*(1 + p.f_cn);                           % + connectors and protection [kg]

% (d) Thermal management, coolant included: specific mass per kW of heat rejected
m_tms = p.k_tms*Q;                                   % [kg] [TEAM]

%% 5. Totals
% (a) Parametric (NASA goals + Eq. 7.21), as in the PDR
m_el_x = m_mot + m_inv + m_inst + m_props + m_wire + m_tms;   % electric system without battery [kg]
m_el   = m_el_x + p.mB;                                       % electric system with battery [kg]
m_prop = m_pg + m_gb + m_el;                                  % parametric propulsion total [kg]
% (b) Itemised: the products selected and the component equations
it_name = {'GE CT7-8A6 turboshaft (dry)', sprintf('Main propeller, %.1f m, %d blades', p.D_prop, p.n_blade), ...
           'Fuel system (integral wing tanks)', 'Starter and engine controls', ...
           'Engine oil and exhaust stub (allowance)', 'Combining gearbox, clutch, HS stage', ...
           'M/G: H3X HPDM-1500 (inverter integral)', sprintf('Blower drives: %d x H3X HPDM-180D', p.n_EP), ...
           sprintf('Blower propellers: %d x %.1f kg', p.n_EP, p.m_EPprop), sprintf('Battery pack (%.0f kWh)', E_nom), ...
           'Trapped fuel', 'HV DC harness and protection', 'Thermal management incl. coolant'};
it_mass = [p.m_eng, m_mainprop, m_fs, m_st + m_ec, p.m_oilexh, m_gb, m_MGunit_tot, m_EPunit_tot, ...
           m_props, p.mB, m_trap, m_wire, m_tms];    % [kg], same order as it_name
it_basis = {'type certificate [TCDS]', sprintf('Dowty R408 scaled by disc area to %.1f m [R408]', p.D_prop), ...
            'Raymer integral-tank equation [RAY]', 'Raymer starter + engine-control equations [RAY]', ...
            'allowance, not in the dry mass [EST]', sprintf('rule: %.2f kg/kW engine+M/G, %.2f kg/kW engine [TEAM/EST]', p.gb_comb, p.gb_hs), ...
            'product data [H3X]', 'product data [H3X]', 'Carbon Voyager class, 24.5 kg + reverse pitch [HZ]', ...
            sprintf('input: %.0f Wh/kg, %.1f kW/kg pack [TEAM]', p.e_b, p.p_b), ...
            sprintf('%.0f%% of the %.0f kg fuel load [EST]', 100*p.f_trap, p.m_fuel_tank), ...
            sprintf('calculated: aluminium at %.0f A/mm^2, cable runs', p.J/1e6), ...
            sprintf('%.1f kg/kW x %.1f kW of heat [TEAM]', p.k_tms, Q)};
m_items  = sum(it_mass);                             % itemised propulsion group [kg]
m_grp_it = p.m_eng + m_mainprop + m_fs + m_st + m_ec + p.m_oilexh;   % engine group, comparable to Eq. 7.21 [kg]

%% 6. Results (every number above, in one struct)
r = struct();
r.inputs = p;
r.P_MG = P_MG;  r.P_sh_cr0 = P_sh_cr0;  r.P_sh_cr = P_sh_cr;  r.LD_cr = LD_cr;  r.t2s = t2s;  r.t2p = t2p;
r.P_ecr = P_ecr;  r.P_cells_cr = P_cells_cr;  r.Q = Q;  r.Q_batt = Q_batt;  r.Q_inv = Q_inv;  r.Q_mot = Q_mot;
r.Q_gb = Q_gb;  r.E_nom = E_nom;  r.P_bmax = P_bmax;
r.m_pg = m_pg;  r.m_gb = m_gb;  r.m_mainprop = m_mainprop;  r.m_fs = m_fs;  r.m_ec = m_ec;  r.m_st = m_st;
r.m_MG = m_MG;  r.m_iMG = m_iMG;  r.m_EPm = m_EPm;  r.m_iEP = m_iEP;  r.m_mot = m_mot;  r.m_inv = m_inv;
r.m_inst = m_inst;  r.m_props = m_props;  r.m_1EP = m_1EP;
r.I_sec = I_sec;  r.I_MG = I_MG;  r.I_EP = I_EP;  r.m_cpack = m_cpack;  r.m_cMG = m_cMG;  r.m_cEP = m_cEP;
r.m_cAC = m_cAC;  r.m_c = m_c;  r.m_wire = m_wire;  r.m_tms = m_tms;
r.m_el_x = m_el_x;  r.m_el = m_el;  r.m_prop = m_prop;
r.m_trap = m_trap;  r.it_name = it_name;  r.it_mass = it_mass;  r.it_basis = it_basis;
r.m_items = m_items;  r.m_grp_it = m_grp_it;

%% 7. Print
if doPrint
    printWeights(r, p);
end
if nargout == 0
    clear('r');                          % no struct dump in the Command Window
end
end

%% ===================================== Local functions =====================================

function p = weightInputs()
% Default inputs: the A7 configuration (07/11/2026). Names match AE481_Propulsion_CT7.m.
p = struct();
% --- Design point, used only for the M/G rating and the cruise heat load ---
p.m_TO     = 7566.9;         % MTOW [kg] [PS]
p.S        = 88.64;        % wing area [m^2] [PS]
p.AR       = 8;            % aspect ratio [PS/IN]
p.e        = 0.825;        % Oswald factor [IN]
p.CD0      = 0.0145;       % clean CD0 at 88.64 m^2 [PS]
p.beta_cr  = 0.97;         % cruise weight / MTOW [IN]
p.V_kt     = 225;          % cruise speed [kt] [RFP]
p.h_cr_ft  = 12000;        % cruise altitude [ft] [TEAM/IN]
p.eta_p    = 0.872;        % cruise propeller efficiency [EST]
p.f_cr     = 0.80;         % cruise at 80% of the M/G rating [TEAM]
p.P_MG_fixed = [];         % [] sizes the M/G from the cruise drag; a number fixes its rating [kW]
p.dD_ram   = 0.022;        % TMS cooling-air drag on the electric legs, fraction of cruise drag [EST]
p.P_aux    = 15;           % non-propulsive electrical load in cruise [kW] [EST, A7 budget]
% --- Efficiencies [TEAM/NASA] ---
p.eta_batt = 0.97;         % battery (cells -> terminals)
p.eta_wire = 0.996;        % wiring
p.eta_conn = 0.999;        % connectors
p.eta_inv  = 0.99;         % inverter
p.eta_tms  = 0.99;         % TMS pumps and fans (charged to the drive chain)
p.eta_mot  = 0.97;         % motor
p.eta_gb   = 0.99;         % gearbox
% --- Engine: GE CT7-8A6 turboshaft [TCDS] ---
p.P_TO     = 2009;         % take-off rating [kW]
p.m_eng    = 245.9;        % dry mass [kg]; excludes starter and mounts
p.k_pg     = 1.16;         % propulsion-group factor, single nose installation [MB]
p.gb_comb  = 0.03;         % combining gearbox + wet clutch [kg per kW of engine + M/G] [TEAM]
p.gb_hs    = 0.02;         % high-speed input stage [kg per kW of engine] [EST]
p.m_oilexh = 10;           % engine oil and exhaust stub, not in the dry mass [kg] [EST allowance]
p.L_ec     = 2.1;          % engine-to-cockpit control run [m]: CT7 at 2.0 m, pilot at 4.1 m [A7]
% --- Main propeller ---
p.D_prop   = 3.0;          % diameter [m] [TEAM]
p.n_blade  = 6;            % blades [TEAM]
p.m_R408   = 252;          % Dowty R408 propeller with spinner [kg] [R408]
p.D_R408   = 4.115;        % its diameter [m] [R408]
% --- Fuel system ---
p.m_fuel_tank = 803.71;    % fuel the tanks hold [kg] [A7: team fuel load incl. reserve]
p.f_trap   = 0.01;         % trapped fuel, fraction of the fuel load [EST]
p.n_tank   = 2;            % fuel tanks [IN]
% --- Electric machines ---
p.P_EP     = 150;          % blown-lift propulsor power [kW] [TEAM]
p.n_EP     = 8;            % number of blown-lift propulsors [TEAM]
p.m_EPprop = 25;           % blower propeller [kg] [HZ: 24.5 kg, +0.5 kg for reverse pitch]
p.m_MGunit = 125;          % H3X HPDM-1500 M/G drive, inverter included [kg] [H3X]
p.m_EPunit = 13.7;         % H3X HPDM-180D blower drive, inverter included [kg] [H3X]
p.sp_mot   = 13;           % parametric motor specific power [kW/kg] [NASA]
p.sp_inv   = 19;           % parametric inverter specific power [kW/kg] [NASA]
p.f_inst   = 0.30;         % parametric installation fraction [TEAM]
% --- Battery ---
p.mB       = 1050;         % battery mass [kg]: the PDR pack (sizing it needs AE481_Propulsion_CT7.m) [TEAM]
p.e_b      = 700;          % pack specific energy [Wh/kg] [TEAM]
p.p_b      = 2.0;          % pack specific power [kW/kg] [TEAM]
p.n_sec    = 2;            % pack sections [TEAM]
% --- Harness: 1,000 V DC, aluminium [TEAM] ---
p.V_bus    = 1000;         % bus voltage [V]
p.J        = 4e6;          % conductor current density [A/m^2] (4 A/mm^2)
p.rho_Al   = 2700;         % aluminium density [kg/m^3]
p.k_route  = 1.3;          % routing factor on straight-line lengths [-]
p.f_ins    = 0.6;          % insulation, fraction of conductor mass [-]
p.f_cn     = 0.3;          % connectors and protection, fraction of cable mass [-]
p.L_pack   = 2.0;          % pack to bus [m]
p.L_MGc    = 3.5;          % bus to M/G [m] [A7/EST]
p.y_EP     = [3.44 5.59 7.74 9.89];   % blower spanwise stations, each side [m] [TEAM]
p.ac_links = false;        % false: inverters inside the motors (H3X), no AC phase cables [H3X]
p.L_ac     = 1.5;          % AC phase cable length, if ac_links [m]
p.V_ac     = 690;          % AC line voltage, if ac_links [V]
p.pf       = 0.95;         % power factor, if ac_links [-]
% --- Thermal management ---
p.k_tms    = 0.5;          % TMS specific mass incl. coolant [kg per kW of heat] [TEAM]
end

function printWeights(r, p)
% Prints the weight breakdown.
LB = 0.45359237;
ln = repmat('-', 1, 92);
fprintf('\nAE481 PROPULSION WEIGHTS (stand-alone; same equations as AE481_Propulsion_CT7.m)\n%s\n', ln);

fprintf('\n1. POWER NUMBERS THE WEIGHTS NEED\n');
fprintf('   225 kt cruise: L/D %.2f, shaft power %.0f kW -> M/G rating %.0f kW (cruise at %.0f%%)\n', ...
    r.LD_cr, r.P_sh_cr0, r.P_MG, 100*p.f_cr);
fprintf('   Electric cruise with TMS drag (+%.1f%%) and %.0f kW aux: %.0f kW at the terminals\n', ...
    100*p.dD_ram, p.P_aux, r.P_ecr);
fprintf('   Heat to reject: battery %.1f + inverter %.1f + motor %.1f + gearbox %.1f = %.1f kW\n', ...
    r.Q_batt, r.Q_inv, r.Q_mot, r.Q_gb, r.Q);
fprintf('   Battery %.0f kg: %.0f kWh, %.0f kW\n', p.mB, r.E_nom, r.P_bmax);

fprintf('\n2. PROPULSION GROUP WEIGHT (products selected + component equations)\n');
for k = 1:numel(r.it_mass)
    fprintf('   %-46s %9.2f kg   %s\n', r.it_name{k}, r.it_mass(k), r.it_basis{k});
end
fprintf('   %-46s %9.2f kg (%.0f lb)\n', 'TOTAL', r.m_items, r.m_items/LB);
fprintf('   %-46s %9.2f kg\n', 'Without the battery', r.m_items - p.mB);

fprintf('\n3. HARNESS DETAIL (%.0f V DC, aluminium, %.1f A/mm^2)\n', p.V_bus, p.J/1e6);
fprintf('   Currents: pack section %.0f A, M/G %.0f A, each blower %.0f A\n', r.I_sec, r.I_MG, r.I_EP);
fprintf('   Pack %.2f + M/G feeder %.2f + blower feeders %.2f + AC links %.2f = %.2f kg; + %.0f%% connectors = %.2f kg\n', ...
    r.m_cpack, r.m_cMG, r.m_cEP, r.m_cAC, r.m_c, 100*p.f_cn, r.m_wire);

fprintf('\n4. PARAMETRIC CHECK (NASA goals + metabook Eq. 7.21, as in the PDR)\n');
fprintf('   Motors %.1f | inverters %.1f | installation %.1f | blower props %.1f | harness %.1f | TMS %.1f kg\n', ...
    r.m_mot, r.m_inv, r.m_inst, r.m_props, r.m_wire, r.m_tms);
fprintf('   Electric system %.1f kg (+ battery = %.1f kg); Eq. 7.21 engine group %.1f kg; gearbox %.1f kg\n', ...
    r.m_el_x, r.m_el, r.m_pg, r.m_gb);
fprintf('   Parametric total %.1f kg; itemised engine group %.1f kg vs Eq. 7.21 %.1f kg\n', ...
    r.m_prop, r.m_grp_it, r.m_pg);

fprintf('%s\n', ln);
end

function rho = atmISA(h, dT)
% ISA troposphere density [kg/m^3] at height h [m], temperature offset dT [K] (pressure of the standard day).
T0 = 288.15;  p0 = 101325;  L = 0.0065;  R = 287.05287;  g = 9.80665;
Ti  = T0 - L*h;                          % standard temperature [K]
pr  = p0*(Ti/T0)^(g/(R*L));              % standard pressure [Pa]
rho = pr/(R*(Ti + dT));                  % density at the offset temperature [kg/m^3]
end