function r = AE481_Propulsion_CT7(varargin)
%AE481_PROPULSION_CT7  M-BRAER hybrid-electric STOL commuter: propulsion numbers for
%   GE CT7-8A6 turboshaft (2035, optimistic) + cruise motor/generator (M/G) + 8 x 150 kW blown-lift EPs.
%
%   AE481_Propulsion_CT7                       A7 configuration: prints the full report (same as pressing Run)
%   r = AE481_Propulsion_CT7;                  the same numbers returned in struct r, nothing printed
%   r = AE481_Propulsion_CT7('P_aux', 30, 'f_ai', 0.193);   change any input of defaultInputs by name
%   AE481_Propulsion_CT7('pdr', true)          the 24/10/2026 PDR configuration
%   r = AE481_Propulsion_CT7('print', true, ...)   print and return the struct
%
% Kit Man Chan, 24/10/2026; updated 06/11/2026, 07/11/2026, 09/11/2026 and 20/11/2026. Plain MATLAB (R2016b or newer),
% no toolboxes. AE481_A7_Checks.m reruns every case quoted in the Assignment 7 write-up.
%
% 20/11/2026: called from run_aircraft_design.m (step 12) through AE481_Propulsion_CT7_Pipeline.m, which
%   passes the final P-S design point. New input cr_from_rating: when the M/G rating is fixed (P_MG_fixed),
%   cruise power now comes from the drag as usual; true restores the PDR assumption (cruise at f_cr x rating).
%   A warning is printed if cruise needs more than the fixed rating.
%
% 09/11/2026: converted from a script to a function, so that cases can be rerun by changing inputs by name.
%   Same equations and input names as the Python version (ae481_propulsion_ct7.py), so both give the same
%   numbers. The inputs are in defaultInputs at the end of this file. Renamed inputs: the TMS ram-air drag
%   is dD_ram (electric legs) and dD_ram_F (fuel leg), formerly dD_E and dD_F. New switches carried over from
%   the Python version: bleed_frac, aux_fl, aux_lo, P_MG_fixed, SOC_div_floor, and 'pdr' for the PDR case.
%
% 07/11/2026 update (Assignment 7: power configuration finalised, refined weights, offtakes):
%   - Battery fixed at the PDR's 1,050 kg (735 kWh); the fuel leg recharges as much as the engine allows.
%   - The M/G tops the battery back up in the climb, where the engine has spare power.
%   - Non-propulsive electrical load 15 kW (was 3 kW), on every phase: from the battery when the engine is
%     off or at idle, from the M/G when the engine drives. 30 kW is the icing case.
%   - Offtakes: TMS ram-air drag (dD_ram, dD_ram_F), installation loss (f_inlet), engine inlet anti-ice
%     (f_ai). No customer bleed is taken. Section 11 checks every P-S constraint with the offtakes deducted.
%   - Itemised hardware masses (H3X drives, Dowty R408 scaled to 3.0 m, Carbon Voyager-class blower
%     propellers, Raymer fuel system, starter and engine controls). The NASA-goal masses are still printed.
%   - Harness without AC phase cables (the H3X units carry the inverter in the motor); M/G feeder 3.5 m.
%
% 06/11/2026 update, to the post-PDR design point (team repo angeete08/Aero481_Team1,
% branch A7CodeRewrite, commit 8cd6c3b):
%   - Design point: MTOW 7,181 kg, S 88.64 m^2, AR 8, CD0 0.0145 (PDR: 5,577 kg, 80.84 m^2, AR 6, 0.0169).
%   - The cruise M/G is sized here from the 225 kt cruise drag at that design point
%     (PDR: 1,334 kW read off the P-S cruise line).
%   - The number of 100 nmi electric legs is an input (n_E). Descent at 1,300 ft/min, as in
%     aircraft_inputs.m. Climb and descent distances are computed (PDR: 16 and 30 nmi typed in).
%   - Main propeller 3.0 m (PDR: 3.4 m). The rpm follows from a fixed 246.4 m/s tip speed, so the
%     helical tip Mach number stays at 0.835 in cruise; the static tip Mach number is printed too.
%   - Battery-only diversion (RFP engine failure at 5,000 ft) is checked at every point of the mission,
%     with feathered-blower drag, the landing energy kept back and the glide from 5,000 ft.
%     keep_res = true sizes the battery to hold that reserve throughout.
%
% Every input states its basis in square brackets:
%   [TCDS] EASA TCDS EASA.IM.E.010, GE CT7 series, Issue 10 (19/12/2025): ratings, mass, size, speed, bleed
%   [T700] Wikipedia, General Electric T700: T700/CT7-family SFC 0.433-0.465 lb/(hp h); front-drive family
%   [TM]   TM 1-1520-238-10 (AH-64A, T700 engines), paras 7.11 and 7A.11: inlet anti-ice on cuts
%          torque available by up to 16% (T700-GE-701) and 19.3% (T700-GE-701C)
%   [CFR]  14 CFR 91.167: IFR fuel = destination + alternate + 45 min at normal cruising speed (aeroplanes)
%   [NASA] Woodworth, NASA EAP keynote (2023): 13 kW/kg motors, 19 kW/kg and 99% inverters;
%          NASA HEMM, LEW-TOPS-140: >1.4 MW, 6,800 rpm, >16 kW/kg, >98% efficiency
%   [H3X]  H3X HPDM-1500 (125 kg) and HPDM-180D (13.7 kg) integrated motor drives, inverter included
%   [R408] EASA TCDS P.002 Issue 3: Dowty R408, 6 composite blades, 4.115 m, 3,782 kW, 252 kg with spinner
%   [HZ]   Hartzell Carbon Voyager: three-blade carbon propeller, 54 lb (24.5 kg) (FLYING, 13/01/2025)
%   [RAY]  Raymer, Aircraft Design, 4th ed., Ch. 15: fuel-system, engine-control and starter equations
%   [MB]   Martins, AE481 metabook, Eq. (7.21): propulsion-group mass
%   [JS]   Johanning & Scholz (2013): propeller efficiency up to about 0.9
%   [PS]   Team P-S code v8, commit 8cd6c3b: selected design 88.64 m^2, 1,347 kW, 7,181 kg;
%          clean CD0 0.0145 at 88.64 m^2 (0.018 at the 62.84 m^2 reference area); constraint powers
%   [IN]   Team single input file aircraft_inputs.m, commit 8cd6c3b
%   [A7]   Team Assignment 7 weight table: usable fuel, trapped fuel, CG stations
%   [TEAM] Team technology set already used in the propulsion section
%   [EST]  Our engineering estimate (reason given where used)

%% 1. Constants
g0 = 9.80665;  KT = 0.514444;  FT = 0.3048;  LB = 0.45359237;  HP = 0.745699872;

%% 2. Inputs: defaults from defaultInputs, then any name-value changes
if mod(numel(varargin), 2) ~= 0
    error('AE481_Propulsion_CT7: give inputs as name-value pairs, for example (''P_aux'', 30)');
end
pdr = false;
doPrint = (nargout == 0);
for k = 1:2:numel(varargin)
    if strcmp(varargin{k}, 'pdr')
        pdr = varargin{k+1};
    elseif strcmp(varargin{k}, 'print')
        doPrint = varargin{k+1};
    end
end
p = defaultInputs(pdr);
for k = 1:2:numel(varargin)
    name = varargin{k};
    if ~any(strcmp(name, {'pdr', 'print'}))
        if ~isfield(p, name)
            error('AE481_Propulsion_CT7: unknown input ''%s'' (see defaultInputs)', name);
        end
        p.(name) = varargin{k+1};
    end
end
assert(any(p.n_E == [1 2]), 'n_E must be 1 or 2');

%% 3. Mission, atmosphere, cruise power and engine power available in cruise
V       = p.V_kt*KT;                         % cruise speed [m/s]
h_cr    = p.h_cr_ft*FT;                      % cruise altitude [m]
R_F     = p.R_des - p.n_E*p.R_E;             % fuel leg [nmi]
ROC_clb = p.ROC_clb_ftmin*0.00508;           % climb rate [m/s]
ROD_dsc = p.ROD_dsc_ftmin*0.00508;           % descent rate [m/s]
if pdr
    R_clb = 16;                              % PDR: climb and descent distances typed in [nmi]
    R_dsc = 30;
else
    R_clb = p.V_clb_kt*p.h_cr_ft/p.ROC_clb_ftmin/60;   % climb distance [nmi], flown on top of the 400 nmi
    R_dsc = p.V_dsc_kt*p.h_cr_ft/p.ROD_dsc_ftmin/60;   % descent distance [nmi], flown on top of the 400 nmi
end
t_clb = (h_cr - 457.2)/ROC_clb/60;           % climb from 1,500 ft [min]
t_dsc = (h_cr - 457.2)/ROD_dsc/60;           % descent to 1,500 ft [min]
K     = 1/(pi*p.AR*p.e);
[rho_cr, T_cr] = atmISA(h_cr, 0);
q_cr  = 0.5*rho_cr*V^2;
W_cr  = p.beta_cr*p.m_TO*g0;                 % cruise weight [N]
CL_cr = W_cr/(q_cr*p.S);
LD_cr = CL_cr/(p.CD0 + K*CL_cr^2);           % cruise L/D at 225 kt, clean airframe
if isempty(p.P_MG_fixed)
    P_sh_cr0 = W_cr/LD_cr*V/p.eta_p/1000;    % propeller shaft power for 225 kt, clean airframe [kW]
    P_MG     = P_sh_cr0/p.f_cr;              % cruise M/G rating [kW]: cruise at f_cr of the rating
else
    P_MG     = p.P_MG_fixed;                 % rating given directly, e.g. read off the P-S cruise line [kW]
    if p.cr_from_rating
        P_sh_cr0 = p.f_cr*P_MG;              % PDR: cruise assumed at f_cr of that rating
    else
        P_sh_cr0 = W_cr/LD_cr*V/p.eta_p/1000;   % cruise power still from the drag [kW]
    end
end
if isempty(p.dD_ram_F)
    dDF = p.dD_ram;                          % fuel leg: same TMS drag as the electric legs
else
    dDF = p.dD_ram_F;
end
P_sh_cr  = P_sh_cr0*(1 + p.dD_ram);          % electric legs, with the TMS ram-air drag [kW]
P_sh_crF = P_sh_cr0*(1 + dDF);               % fuel leg, with the TMS ram-air drag [kW]
rho0   = 1.225;
a_cr   = sqrt(1.4*287.05287*T_cr);
M_cr   = V/a_cr;
lapse  = (rho_cr/rho0)^0.75*((1 + 0.2*M_cr^2)^3.5)^0.75;   % sigma^0.75 with ram recovery
P_cont = p.P_TO;                             % OPTIMISTIC: the 2035 derivative runs its take-off rating
                                             % continuously (the CT7-9 turboprop already does) [EST]
P_av   = min(P_cont, p.P_TO*lapse);          % engine power available in cruise [kW]

%% 4. Efficiency chain
t2s   = p.eta_wire*p.eta_conn*p.eta_inv*p.eta_tms*p.eta_mot;   % battery terminals -> motor shaft
t2p   = t2s*p.eta_gb;                                           % -> propeller shaft
chain = p.eta_batt*t2p*p.eta_p;                                 % battery cells -> thrust power in cruise

%% 5. Mission energy and battery (blowers on the battery, no M/G boost at take-off)
% Power schedule: take-off and initial climb on the engine + blowers (battery); climb on the engine, with the
% M/G generating to top the battery back up; E1 on the M/G (engine off); restart; fuel leg on the engine with
% the M/G recharging at the most the engine allows; E2; restart; idle descent; blown approach and landing.
hr       = @(nmi) nmi/p.V_kt;                % hours at cruise speed
P_TOterm = p.n_EP*p.P_EP/t2s;                % take-off: blowers at full power [kW at the terminals]
P_part   = 0.5*p.n_EP*p.P_EP/t2s;            % initial climb and approach: blowers at 50%
P_ecr    = P_sh_cr/t2p + p.P_aux;            % electric cruise [kW at the terminals]
P_gbin   = P_sh_crF/p.eta_gb;                % engine output to hold 225 kt on the fuel leg [kW]
P_lo     = p.P_aux*p.aux_lo;                 % aux load on the battery in take-off, restarts, descent, landing
aux_gen  = p.P_aux*p.aux_fl;                 % aux load on the M/G while the engine drives (climb, fuel leg)
E_TO     = (P_TOterm + P_lo)*p.t_TO/60/p.eta_batt;
E_IC     = (P_part + P_lo)*p.t_IC/60/p.eta_batt;
E_leg    = P_ecr*hr(p.R_E)/p.eta_batt;
E_rst    = P_lo*p.t_rst/60/p.eta_batt;       % each restart at flight idle
E_dsc    = P_lo*t_dsc/60/p.eta_batt;         % idle descent
E_land   = (P_part + P_lo)*p.t_app/60/p.eta_batt;
P_av_eff = max(0, P_av*(1 - p.f_inlet)*(1 - p.bleed_frac) - p.f_ai*p.P_TO);   % after installation loss,
                                                                             % bleed and inlet anti-ice [kW]
% Climb: the engine at its rating has spare power, so the M/G generates and puts the take-off energy back
[rho_m, T_m] = atmISA((457.2 + h_cr)/2, 0); % mid-climb
Vc  = p.V_clb_kt*KT;
Wt  = p.m_TO*g0;
qc  = 0.5*rho_m*Vc^2;
CLc = Wt/(qc*p.S);
P_clb    = (qc*p.S*(p.CD0 + K*CLc^2)*Vc + Wt*ROC_clb)/p.eta_clb/1000/p.eta_gb;   % engine output, climb [kW]
M_c      = Vc/sqrt(1.4*287.05287*T_m);
lapse_c  = (rho_m/1.225)^0.75*((1 + 0.2*M_c^2)^3.5)^0.75;
P_avc    = max(0, min(p.P_TO, p.P_TO*lapse_c)*(1 - p.f_inlet)*(1 - p.bleed_frac) - p.f_ai*p.P_TO);
P_topmax = max(0, min(P_MG, max(0, P_avc - P_clb))*t2s - aux_gen);   % charge power in the climb [kW]
if p.topup_clb
    E_top = min(E_TO + E_IC, P_topmax*t_clb/60*p.eta_batt);          % energy put back in the climb [kWh]
else
    E_top = 0;
end
if E_top > 0
    t_top = 60*E_top/(p.eta_batt*P_topmax);  % [min]
else
    t_top = 0;
end
E_A      = E_TO + E_IC - E_top + E_leg + E_rst;              % net energy used before the fuel leg [kWh]
E_after  = (p.n_E - 1)*(E_leg + E_rst) + E_dsc + E_land;     % energy used after the fuel leg [kWh]
P_chgmax = max(0, min(P_MG, max(0, P_av_eff - P_gbin))*t2s - aux_gen);   % recharge at the terminals; the M/G
                                                                         % also carries the aux load [kW]
E_inmax  = min(P_chgmax*hr(R_F)*p.eta_batt, E_A);           % most the fuel leg can put back [kWh]
% Battery-only diversion: best-range speed at 5,000 ft, ISA+18 F, MTOW, blowers feathered,
% main propeller driven by the M/G with the engine declutched
rho5   = atmISA(5000*FT, 10);
CD0_e  = p.CD0 + p.dCD0_fth;
CLs    = sqrt(CD0_e/K);
LDmax  = 1/(2*sqrt(CD0_e*K));
Vbr    = sqrt(2*Wt/(rho5*p.S*CLs));
Pbr    = Wt*Vbr/LDmax/p.eta_lo/1000;         % best-range shaft power [kW]
P_cells_br = (Pbr/t2p + p.P_aux)/p.eta_batt; % battery cell power at best range [kW]
E_per_nmi  = P_cells_br/(Vbr/KT);            % cell energy per nmi at best range [kWh/nmi]
R_glide = p.h_emerg_ft*FT*LDmax/1852;        % distance gained gliding down from the failure height [nmi]
E_res   = max(0, p.R_emerg_req - R_glide)*E_per_nmi + E_land;   % diversion + blown landing reserve [kWh]
E_keep  = p.keep_res*E_res/p.f_EOL;          % reserve held back from the mission [kWh]
E_need  = max(E_A, E_A - E_inmax + E_after) + E_keep;   % usable energy the pack must hold [kWh]
mB_E    = 1000*E_need/((1 - p.SOC_min)*p.e_b);
P_peak  = max(P_TOterm, P_ecr);
mB_P    = P_peak/p.p_b;
if isempty(p.mB_fix)
    mB = ceil(max(mB_E, mB_P)/10)*10;        % battery sized here, rounded up to 10 kg
else
    mB = p.mB_fix;                           % battery fixed
end
E_nom  = mB*p.e_b/1000;
P_bmax = mB*p.p_b;
E_use  = (1 - p.SOC_min)*E_nom;
if p.rech_max
    E_in = E_inmax;                                                 % recharge as much as the fuel leg allows
else
    E_in = min(E_inmax, max(0, E_A + E_after - (E_use - E_keep)));  % only the top-up needed
end
P_chg = E_in/(p.eta_batt*hr(R_F));           % recharge at the terminals [kW]
P_fl  = P_gbin + (P_chg + aux_gen)/t2s;      % engine output on the fuel leg [kW]
soc = [1, 1 - E_TO/E_nom, 1 - (E_TO + E_IC)/E_nom, 1 - (E_TO + E_IC - E_top)/E_nom, 1 - E_A/E_nom, 1 - (E_A - E_in)/E_nom];
lbl = {'start', 'after take-off', 'after initial climb', 'top of climb', 'after E1 and restart', 'after fuel leg'};
if p.n_E == 2
    soc(end+1) = 1 - (E_A - E_in + E_leg + E_rst)/E_nom;
    lbl{end+1} = 'after E2 and restart';
end
soc(end+1) = 1 - (E_A - E_in + E_after)/E_nom;
lbl{end+1} = 'at landing';
P_worst = P_MG/t2s + P_TOterm + p.P_aux;     % M/G at full rating + blowers + aux (engine failure at take-off)

%% 6. Fuel
cp    = p.cp_lb*LB/HP;                       % [kg/kWh]
ff_TO = cp*p.P_TO;                           % [kg/h]
ff_id = p.f_idle*ff_TO;                      % [kg/h]
ph_name = {'Taxi-out, start and warm-up (idle)', 'Take-off (take-off rating)', 'Initial climb (take-off rating)', 'Climb to cruise altitude'};
ph_ff   = [ff_id, ff_TO, ff_TO, cp*(P_clb + aux_gen/t2s)];   % fuel flow [kg/h]
ph_t    = [p.t_taxo, p.t_TO, p.t_IC, t_clb];                 % time [min]
if E_top > 0
    ph_name{end+1} = 'Climb: battery top-up (M/G generating)';
    ph_ff(end+1)   = cp*P_topmax/t2s;
    ph_t(end+1)    = t_top;
end
ph_name{end+1} = 'Electric leg 1 (engine off)';
ph_name{end+1} = 'Restart and warm-up (flight idle)';
ph_name{end+1} = 'Fuel leg, propulsion + recharge';
ph_ff = [ph_ff, 0, ff_id, cp*P_fl];
ph_t  = [ph_t, hr(p.R_E)*60, p.t_rst, hr(R_F)*60];
if p.n_E == 2
    ph_name{end+1} = 'Electric leg 2 (engine off)';
    ph_name{end+1} = 'Restart before descent (flight idle)';
    ph_ff = [ph_ff, 0, ff_id];
    ph_t  = [ph_t, hr(p.R_E)*60, p.t_rst];
end
ph_name{end+1} = 'Descent (flight idle)';
ph_name{end+1} = 'Blown approach and landing (idle)';
ph_name{end+1} = 'Taxi-in (idle)';
ph_ff = [ph_ff, ff_id, ff_id, ff_id];
ph_t  = [ph_t, t_dsc, p.t_app, p.t_taxi];
fuel_ph = ph_ff.*ph_t/60;                    % fuel per phase [kg]
m_block = sum(fuel_ph);                      % design mission fuel burn
m_res45 = cp*(P_gbin + aux_gen/t2s)*(hr(p.R_alt) + p.t_res/60);   % IFR reserve flown at normal cruise speed
m_res30 = cp*(P_gbin + aux_gen/t2s)*(hr(p.R_alt) + 30/60);        % same with the 30 min in aircraft_inputs.m
m_fuel  = m_block + m_res45;
m_chg   = cp*(P_fl - P_gbin - aux_gen/t2s)*hr(R_F) + cp*E_top/(p.eta_batt*t2s);   % fuel burned to recharge

%% 7. Ranges
P_cells_cr = P_ecr/p.eta_batt;
R_el_cont  = E_use/P_cells_cr*p.V_kt;        % continuous electric, 100% -> 10%, 225 kt, cruise altitude [nmi]
E_emerg    = E_use - (E_TO + E_IC);          % above the floor after take-off and initial climb [kWh]
q5  = 0.5*rho5*V^2;                          % RFP engine-failure case: 5,000 ft, ISA+18 F
CL5 = Wt/(q5*p.S);
P5  = q5*p.S*(CD0_e + K*CL5^2)*V/p.eta_p/1000;   % shaft power for 225 kt at 5,000 ft [kW]
R_eo225 = E_emerg/((P5/t2p + p.P_aux)/p.eta_batt)*p.V_kt;
R_eobr  = E_emerg/P_cells_br*(Vbr/KT);       % at best-range (L/D max) speed
% Diversion range from every point of the mission: charge above the floor, landing energy kept back,
% plus the glide from 5,000 ft
if isempty(p.SOC_div_floor)
    fl = p.SOC_min;                          % keep SOC_min in a diversion
else
    fl = p.SOC_div_floor;                    % e.g. 0: the floor may be used in an emergency
end
R_div     = max(0, p.f_EOL*(soc - fl)*E_nom - E_land)/E_per_nmi + R_glide;
R_glide12 = h_cr*LDmax/1852;                 % glide from cruise altitude, where the lowest charge occurs [nmi]

%% 8. Thrust and TSFC (actuator disc with figure of merit, static, ISA sea level)
A     = pi*p.D_prop^2/4;
T0    = @(P_kW, Ad) (2*rho0*Ad)^(1/3)*(p.FM*P_kW*1e3)^(2/3);
T_TO  = T0(p.P_TO*p.eta_gb, A);              % take-off rating
T_max = T0(p.P_30s*p.eta_gb, A);             % 30-s contingency rating
T_EMG = T0((p.P_TO + P_MG)*p.eta_gb, A);     % engine + M/G (not used: no take-off boost)
T_EP  = T0(p.P_EP*p.eta_gb, pi*p.D_EP^2/4);  % one blower
cpJ     = cp/3.6e6;                          % [kg/J]
TSFC_cr = cpJ*g0*V/p.eta_p;                  % cruise [1/s]
TSFC_0  = cpJ*p.P_TO*1e3*g0/T_TO;            % static, take-off [1/s]

%% 9. Masses
% (a) Parametric, NASA technology goals (as in the PDR)
m_MG    = P_MG/p.sp_mot;
m_iMG   = P_MG/p.eta_mot/p.sp_inv;
m_EPm   = p.P_EP/p.sp_mot;
m_iEP   = p.P_EP/p.eta_mot/p.sp_inv;
m_mot   = m_MG + p.n_EP*m_EPm;
m_inv   = m_iMG + p.n_EP*m_iEP;
m_inst  = p.f_inst*(m_mot + m_inv);
m_props = p.n_EP*p.m_EPprop;
m_1EP   = (m_EPm + m_iEP)*(1 + p.f_inst) + p.m_EPprop;   % one blown-lift propulsor
P_aw = (P_ecr - p.P_aux)*p.eta_wire*p.eta_conn;          % heat at the electric-cruise condition
Q = (P_cells_cr - P_ecr) + P_aw*(1 - p.eta_inv) + P_aw*p.eta_inv*p.eta_tms*(1 - p.eta_mot) ...
    + P_aw*p.eta_inv*p.eta_tms*p.eta_mot*(1 - p.eta_gb);
m_tms = p.k_tms*Q;
cableMass = @(I, L, n, nc) n*nc*p.k_route*L*(I/p.J)*p.rho_Al*(1 + p.f_ins);   % n runs of nc conductors
I_sec = P_bmax/p.n_sec/p.V_bus*1e3;          % each pack section at full pack power [A]
I_MG  = P_MG/p.eta_mot/p.eta_inv/p.V_bus*1e3;
I_EP  = p.P_EP/p.eta_mot/p.eta_inv/p.V_bus*1e3;
m_c = cableMass(I_sec, p.L_pack, p.n_sec, 2) + cableMass(I_MG, p.L_MGc, 1, 2);
for y = p.y_EP
    m_c = m_c + cableMass(I_EP, y, 2, 2);
end
if p.ac_links                                % separate inverters: short AC phase cables to each motor
    I_acMG = P_MG/p.eta_mot/(sqrt(3)*p.V_ac*p.pf)*1e3;
    I_acEP = p.P_EP/p.eta_mot/(sqrt(3)*p.V_ac*p.pf)*1e3;
    m_c = m_c + cableMass(I_acMG, p.L_ac/p.k_route, 1, 3) + cableMass(I_acEP, p.L_ac/p.k_route, p.n_EP, 3);
end
m_wire = m_c*(1 + p.f_cn);
m_el_x = m_mot + m_inv + m_inst + m_props + m_wire + m_tms;   % electric system without battery
m_el   = m_el_x + mB;
m_pg   = p.k_pg*(p.m_eng/LB + 0.24*p.P_TO/HP)*LB;             % Eq. 7.21: engine, main prop, installation
m_gb   = p.gb_comb*(p.P_TO + P_MG) + p.gb_hs*p.P_TO;
m_prop = m_pg + m_gb + m_el;
% (b) Itemised hardware (A7): the products selected and component equations
W_en_lb    = p.m_eng/LB;
m_mainprop = p.m_R408*(p.D_prop/p.D_R408)^2*(p.n_blade/6);    % R408 scaled by disc area [R408]
Vt_gal     = (p.m_fuel_tank + p.m_trap)/0.8/3.785411784;      % tank volume at 0.8 kg/L [US gal]
m_fs = 2.49*Vt_gal^0.726*(1/(1 + 1))^0.363*p.n_tank^0.242*1^0.157*LB;   % fuel system, integral tanks [RAY]
m_ec = (5.0*1 + 0.80*p.L_ec/FT)*LB;          % engine controls, 1 engine [RAY]
m_st = 49.19*(1*W_en_lb/1000)^0.541*LB;      % starter, 1 engine [RAY]
it_name = {'GE CT7-8A6 turboshaft (dry)', sprintf('Main propeller, %.1f m, %d blades', p.D_prop, p.n_blade), ...
           'Fuel system (integral wing tanks)', 'Starter and engine controls', ...
           'Engine oil and exhaust stub (allowance)', 'Combining gearbox, clutch, HS stage', ...
           'M/G: H3X HPDM-1500 (inverter integral)', sprintf('Blower drives: %d x H3X HPDM-180D', p.n_EP), ...
           sprintf('Blower propellers: %d x %.1f kg', p.n_EP, p.m_EPprop), sprintf('Battery pack (%.0f kWh)', E_nom), ...
           'Trapped fuel', 'HV DC harness and protection', 'Thermal management incl. coolant'};
it_mass = [p.m_eng, m_mainprop, m_fs, m_st + m_ec, p.m_oilexh, m_gb, p.m_MGunit, p.n_EP*p.m_EPunit, ...
           p.n_EP*p.m_EPprop, mB, p.m_trap, m_wire, m_tms];
m_items  = sum(it_mass);
m_grp_it = p.m_eng + m_mainprop + m_fs + m_st + m_ec + p.m_oilexh;   % itemised counterpart of Eq. 7.21

%% 10. Layout numbers
N_prop = 60*p.V_tip/(pi*p.D_prop);           % propeller speed [rpm]: 1,569 rpm at 3.0 m
Vtip   = pi*p.D_prop*N_prop/60;              % tip speed from rotation [m/s]
M_hel  = sqrt(Vtip^2 + V^2)/a_cr;            % helical tip Mach in cruise (rotation + flight speed)
[~, T_hot] = atmISA(0, 10);                  % sea level, ISA+18 F (RFP field condition)
M_tip0 = Vtip/sqrt(1.4*287.05287*T_hot);     % static tip Mach at take-off
i_eng  = p.N_eng/N_prop;
i_MG   = p.N_MG/N_prop;
N_EPprop = p.V_tipEP/(pi*p.D_EP)*60;
N_EPmot  = p.i_EP*N_EPprop;
[D_MGm, L_MGm] = motorEnvelope(P_MG, p.N_MG, p.sigma_gap);
[D_EPm, L_EPm] = motorEnvelope(p.P_EP, N_EPmot, p.sigma_gap);
[D_EPd, L_EPd] = motorEnvelope(p.P_EP, N_EPprop, p.sigma_gap);   % direct-drive alternative
V_pack   = mB/p.rho_pack/1000;               % [m^3]
P_el_tot = P_MG + p.n_EP*p.P_EP;

%% 11. Offtakes and the feasibility envelope
% Cooling air for the TMS (ram air) and the momentum drag if all of its momentum were lost
mdot_hx = Q*1e3/(1005*p.dT_hx);              % [kg/s]
D_ram   = mdot_hx*V;                         % [N]
D_cr    = W_cr/LD_cr;                        % clean cruise drag [N]
% Shaft power available at the propellers for each P-S constraint, net of offtakes [kW]
sig_hot = atmISA(0, 10)/rho0;
P_SL    = min(P_cont, p.P_TO*sig_hot^0.75)*(1 - p.f_inlet);        % engine, SL ISA+10 (no icing at ISA+10)
P_SLi   = (p.P_TO - p.f_ai_ice*p.P_TO)*(1 - p.f_inlet);            % engine, cold SL day, inlet anti-ice on
P_clot  = min(P_bmax - p.P_aux, P_MG/t2s + P_TOterm)*t2s;          % engine failed: battery drives M/G + blowers
P_cloti = min(P_bmax - p.P_aux_ice, P_MG/t2s + P_TOterm)*t2s;
P_F     = (P_av*(1 - p.f_inlet) - p.P_aux/t2s)*p.eta_gb;           % fuel leg: engine at 12,000 ft, aux on the M/G
P_Fi    = (P_av*(1 - p.f_inlet) - p.f_ai_ice*p.P_TO - p.P_aux_ice/t2s)*p.eta_gb;
P_E     = P_MG*p.eta_gb;                                           % electric legs: M/G at its rating
rho_c   = atmISA(12500*FT, 0);
P_ce    = min(P_cont, p.P_TO*(rho_c/rho0)^0.75)*(1 - p.f_inlet);   % engine at the ceiling
P_C     = (P_ce - p.P_aux/t2s)*p.eta_gb;
P_Ci    = (P_ce - p.f_ai_ice*p.P_TO - p.P_aux_ice/t2s)*p.eta_gb;
dF = 1 + dDF;
dE = 1 + p.dD_ram;
PS = p.PS_need;
chk_name = {'Take-off, 300 ft (SL, ISA+10)', 'RFP climb 1,500 ft/min (SL, ISA+10)', 'Part 23 initial climb, 4%', ...
            'Climb after engine failure, 1%', 'Balked landing, 3%', '225 kt cruise, fuel leg', '225 kt cruise, electric leg', ...
            '30 deg bank, 225 kt, 12,000 ft, fuel leg', '30 deg bank, 225 kt, 12,000 ft, electric leg', ...
            'Ceiling 12,500 ft, 100 ft/min', 'Objective 250 kt cruise, fuel leg'};
chk_src  = {'engine + 8 blowers', 'engine', 'engine', 'battery: M/G + blowers', 'engine', 'engine', 'M/G', ...
            'engine', 'M/G', 'engine', 'engine'};
chk_need = [PS(1:5), PS(6)*dF, PS(6)*dE, PS(7)*dF, PS(7)*dE, PS(8)*dF, PS(9)*dF];
chk_av   = [P_SL*p.eta_gb + p.n_EP*p.P_EP, P_SL*p.eta_gb, P_SL*p.eta_gb, P_clot, P_SL*p.eta_gb, P_F, P_E, P_F, P_E, P_C, P_F];
chk_avi  = [P_SLi*p.eta_gb + p.n_EP*p.P_EP, P_SLi*p.eta_gb, P_SLi*p.eta_gb, P_cloti, P_SLi*p.eta_gb, P_Fi, NaN, P_Fi, NaN, P_Ci, P_Fi];
chk_mar  = chk_av./chk_need - 1;             % power margin, normal
chk_mari = chk_avi./chk_need - 1;            % power margin, icing (NaN: not flown in icing)

%% 12. Results
r = struct();
r.inputs = p;  r.pdr = pdr;
r.V = V;  r.h_cr = h_cr;  r.R_F = R_F;  r.R_clb = R_clb;  r.R_dsc = R_dsc;  r.t_clb = t_clb;  r.t_dsc = t_dsc;  r.K = K;
r.rho_cr = rho_cr;  r.CL_cr = CL_cr;  r.LD_cr = LD_cr;  r.W_cr = W_cr;  r.D_cr = D_cr;  r.a_cr = a_cr;  r.M_cr = M_cr;
r.P_sh_cr0 = P_sh_cr0;  r.P_MG = P_MG;  r.dDF = dDF;  r.P_sh_cr = P_sh_cr;  r.P_sh_crF = P_sh_crF;
r.lapse = lapse;  r.P_cont = P_cont;  r.P_av = P_av;  r.P_av_eff = P_av_eff;  r.P_gbin = P_gbin;
r.t2s = t2s;  r.t2p = t2p;  r.chain = chain;
r.P_TOterm = P_TOterm;  r.P_part = P_part;  r.P_ecr = P_ecr;  r.P_lo = P_lo;  r.aux_gen = aux_gen;
r.E_TO = E_TO;  r.E_IC = E_IC;  r.E_leg = E_leg;  r.E_rst = E_rst;  r.E_dsc = E_dsc;  r.E_land = E_land;
r.P_clb = P_clb;  r.P_avc = P_avc;  r.P_topmax = P_topmax;  r.E_top = E_top;  r.t_top = t_top;
r.E_A = E_A;  r.E_after = E_after;  r.P_chgmax = P_chgmax;  r.E_inmax = E_inmax;
r.LDmax = LDmax;  r.Vbr_kt = Vbr/KT;  r.P_cells_br = P_cells_br;  r.E_per_nmi = E_per_nmi;  r.R_glide = R_glide;
r.R_glide12 = R_glide12;  r.E_res = E_res;  r.E_keep = E_keep;  r.E_need = E_need;  r.mB_E = mB_E;  r.mB_P = mB_P;
r.mB = mB;  r.E_nom = E_nom;  r.P_bmax = P_bmax;  r.E_use = E_use;  r.E_in = E_in;  r.P_chg = P_chg;  r.P_fl = P_fl;
r.soc = soc;  r.lbl = lbl;  r.P_worst = P_worst;
r.cp = cp;  r.ph_name = ph_name;  r.ph_ff = ph_ff;  r.ph_t = ph_t;  r.fuel_ph = fuel_ph;  r.m_block = m_block;
r.m_res45 = m_res45;  r.m_res30 = m_res30;  r.m_fuel = m_fuel;  r.m_chg = m_chg;
r.P_cells_cr = P_cells_cr;  r.R_el_cont = R_el_cont;  r.E_emerg = E_emerg;  r.R_eo225 = R_eo225;  r.R_eobr = R_eobr;
r.R_div = R_div;
r.T_TO = T_TO;  r.T_max = T_max;  r.T_EMG = T_EMG;  r.T_EP = T_EP;  r.TSFC_cr = TSFC_cr;  r.TSFC_0 = TSFC_0;
r.m_MG = m_MG;  r.m_iMG = m_iMG;  r.m_EPm = m_EPm;  r.m_iEP = m_iEP;  r.m_mot = m_mot;  r.m_inv = m_inv;
r.m_inst = m_inst;  r.m_props = m_props;  r.m_1EP = m_1EP;  r.Q = Q;  r.m_tms = m_tms;
r.I_sec = I_sec;  r.I_MG = I_MG;  r.I_EP = I_EP;  r.m_wire = m_wire;  r.m_el_x = m_el_x;  r.m_el = m_el;
r.m_pg = m_pg;  r.m_gb = m_gb;  r.m_prop = m_prop;
r.m_mainprop = m_mainprop;  r.m_fs = m_fs;  r.m_ec = m_ec;  r.m_st = m_st;
r.it_name = it_name;  r.it_mass = it_mass;  r.m_items = m_items;  r.m_grp_it = m_grp_it;
r.N_prop = N_prop;  r.Vtip = Vtip;  r.M_hel = M_hel;  r.M_tip0 = M_tip0;  r.i_eng = i_eng;  r.i_MG = i_MG;
r.N_EPprop = N_EPprop;  r.N_EPmot = N_EPmot;  r.D_MGm = D_MGm;  r.L_MGm = L_MGm;  r.D_EPm = D_EPm;  r.L_EPm = L_EPm;
r.D_EPd = D_EPd;  r.L_EPd = L_EPd;  r.V_pack = V_pack;  r.P_el_tot = P_el_tot;
r.mdot_hx = mdot_hx;  r.D_ram = D_ram;  r.sig_hot = sig_hot;
r.P_SL = P_SL;  r.P_SLi = P_SLi;  r.P_clot = P_clot;  r.P_cloti = P_cloti;  r.P_F = P_F;  r.P_Fi = P_Fi;  r.P_E = P_E;
r.P_C = P_C;  r.P_Ci = P_Ci;
r.chk_name = chk_name;  r.chk_src = chk_src;  r.chk_need = chk_need;  r.chk_av = chk_av;  r.chk_avi = chk_avi;
r.chk_mar = chk_mar;  r.chk_mari = chk_mari;

%% 13. Print
if doPrint
    printReport(r, p);
end
if nargout == 0
    clear('r');                              % no struct dump in the Command Window
end
end

%% ===================================== Local functions =====================================

function p = defaultInputs(pdr)
% Inputs of the A7 configuration (07/11/2026); pdr = true changes them to the 24/10/2026 PDR values.
p = struct();
% --- Mission ---
p.V_kt     = 225;          % cruise speed [kt] [RFP threshold]
p.h_cr_ft  = 12000;        % cruise altitude [ft] [TEAM/IN]
p.R_des    = 400;          % design range, counted as cruise distance only [nmi] [RFP/TEAM]
p.R_E      = 100;          % each electric leg [nmi] [TEAM]
p.n_E      = 2;            % electric legs: 2 = E1 + fuel leg + E2, 1 = E1 + fuel leg [TEAM]
p.t_taxo   = 5;            % taxi-out incl. start and warm-up [min] [TEAM/EST]
p.t_TO     = 1;            % take-off [min] [TEAM/EST]
p.t_IC     = 2;            % initial climb [min] [TEAM/EST]
p.t_rst    = 2;            % restart + warm-up at flight idle, after each electric leg [min] [TEAM]
p.t_app    = 1.5;          % blown approach and landing [min] [TEAM/EST]
p.t_taxi   = 3;            % taxi-in [min] [TEAM/EST]
p.R_alt    = 50;           % IFR alternate [nmi] [TEAM]
p.t_res    = 45;           % IFR reserve at cruise speed [min] [CFR]
p.V_clb_kt = 120;          % en-route climb speed [kt] [TEAM/IN]
p.ROC_clb_ftmin = 1500;    % climb rate [ft/min] [TEAM/IN]
p.V_dsc_kt = 150;          % descent speed [kt] [IN]
p.ROD_dsc_ftmin = 1300;    % descent rate [ft/min] [IN]
% --- Aircraft at the P-S design point (commit 8cd6c3b) ---
p.m_TO     = 7181;         % MTOW [kg] [PS]
p.S        = 88.64;        % wing area [m^2] [PS]
p.AR       = 8;            % aspect ratio [PS/IN]
p.e        = 0.825;        % Oswald factor [IN]
p.CD0      = 0.0145;       % clean CD0 at 88.64 m^2 (0.018 at 62.84 m^2) [PS]
p.beta_cr  = 0.97;         % cruise weight / MTOW [IN: in.beta_cruise]
p.f_cr     = 0.80;         % cruise at 80% of the M/G rating [TEAM]
p.P_MG_fixed = [];         % [] sizes the M/G from the cruise drag; a number fixes its rating [kW]
p.cr_from_rating = false;  % with P_MG_fixed: true takes cruise power as f_cr x rating (PDR); false from drag
% --- Engine: GE CT7-8A6 turboshaft [TCDS] ---
p.P_TO     = 2009;         % take-off rating (5 min) [kW]
p.P_MCP    = 1736;         % maximum continuous [kW]
p.P_30s    = 2122;         % 30-s OEI [kW]
p.m_eng    = 245.9;        % dry mass [kg]; excludes starter and mounts
p.L_eng    = 1.239;        % overall length [m]
p.W_eng    = 0.660;        % width [m]
p.H_eng    = 0.635;        % height [m]
p.N_eng    = 21945;        % output (power-turbine) speed [rpm]
p.bleed_max = 0.065;       % customer bleed allowed, fraction of core airflow; we take none [TCDS]
p.cp_lb    = 0.40;         % 2035 SFC [lb/(hp h)]; T700/CT7 family is 0.433-0.465 today [T700]
p.f_idle   = 0.25;         % idle fuel flow as a fraction of take-off fuel flow (conservative) [TEAM]
p.k_pg     = 1.16;         % propulsion-group factor, single nose installation [MB]
p.gb_comb  = 0.03;         % combining gearbox + wet clutch [kg per kW of engine + M/G] [TEAM]
p.gb_hs    = 0.02;         % extra high-speed input stage for the 21,945 rpm shaft [kg per kW of engine] [EST]
% --- Main propeller ---
p.D_prop   = 3.0;          % diameter [m] [TEAM]
p.V_tip    = 246.4;        % tip speed [m/s]: the PDR's 3.4 m at 1,384 rpm (CT7-9 propeller speed) [TEAM]
p.n_blade  = 6;            % blades [TEAM]
p.FM       = 0.75;         % static figure of merit [TEAM]
p.eta_p    = 0.872;        % cruise propeller efficiency: momentum theory with the PDR's blade losses [EST/JS]
p.eta_clb  = 0.80;         % climb propeller efficiency [TEAM]
p.eta_lo   = 0.80;         % low-speed propeller efficiency [TEAM]
% --- Electric system ---
p.P_EP     = 150;          % blown-lift propulsor power [kW] [TEAM]
p.n_EP     = 8;            % number of blown-lift propulsors [TEAM]
p.D_EP     = 1.98;         % EP propeller diameter [m] [TEAM]
p.V_tipEP  = 200;          % EP static tip speed [m/s] [EST]
p.i_EP     = 4;            % EP motor:propeller speed ratio [EST]
p.y_EP     = [3.44 5.59 7.74 9.89];   % EP spanwise stations, each side [m] [TEAM]
p.N_MG     = 6800;         % M/G speed [rpm], NASA HEMM rated speed [NASA]
p.sp_mot   = 13;           % motor specific power [kW/kg] [NASA]
p.sp_inv   = 19;           % inverter specific power [kW/kg] [NASA]
p.f_inst   = 0.30;         % installation fraction [TEAM]
p.m_EPprop = 25;           % EP propeller [kg] [HZ: 24.5 kg, +0.5 kg for reverse pitch]
p.P_aux    = 15;           % non-propulsive electrical load [kW] [EST, A7 budget]
p.P_aux_ice = 30;          % same in icing [kW] [EST, A7 budget]
p.k_tms    = 0.5;          % TMS [kg per kW of heat] [TEAM]
p.eta_batt = 0.97;  p.eta_wire = 0.996;  p.eta_conn = 0.999;  p.eta_inv = 0.99;   % [TEAM/NASA]
p.eta_tms  = 0.99;  p.eta_mot  = 0.97;   p.eta_gb   = 0.99;                       % [TEAM/NASA]
p.e_b      = 700;          % pack specific energy [Wh/kg] [TEAM]
p.p_b      = 2.0;          % pack specific power [kW/kg] [TEAM]
p.SOC_min  = 0.10;         % lowest state of charge [-] [TEAM]
p.rho_pack = 2.0;          % pack density [kg/L] [TEAM]
p.n_sec    = 2;            % pack sections [TEAM]
p.sigma_gap = 60e3;        % motor air-gap shear stress for envelope sizing [Pa] [EST]
p.V_bus    = 1000;         % DC bus voltage [V] [TEAM]
p.J        = 4e6;          % conductor current density [A/m^2] (4 A/mm^2) [TEAM]
p.rho_Al   = 2700;         % aluminium density [kg/m^3]
p.k_route  = 1.3;          % routing factor on straight-line cable lengths [TEAM]
p.f_ins    = 0.6;          % insulation, fraction of conductor mass [TEAM]
p.f_cn     = 0.3;          % connectors and protection, fraction of cable mass [TEAM]
p.L_pack   = 2.0;          % pack to bus [m] [TEAM]
p.L_MGc    = 3.5;          % battery to M/G, A7 stations [m] [A7/EST]
p.L_ac     = 1.5;          % AC phase cable, inverter to motor [m] [TEAM]
p.V_ac     = 690;          % AC line voltage [V] [TEAM]
p.pf       = 0.95;         % power factor [TEAM]
p.ac_links = false;        % false: inverters inside the motors (H3X), so no separate AC phase cables [H3X]
% --- Battery-only diversion after an engine failure (RFP: 5,000 ft AGL, ISA+18 F, max weight) ---
p.R_emerg_req = 50;        % required diversion to an alternate [nmi]: same 50 nmi as the IFR alternate [TEAM]
p.h_emerg_ft  = 5000;      % height at the failure [ft] [RFP]
p.dCD0_fth = 0.0015;       % feathered, locked blower propellers [-] [TEAM: 0.001-0.002 for 3-5 blades]
p.f_EOL    = 1.0;          % battery capacity left at end of life (0.8 for a conservative check) [EST]
p.keep_res = false;        % true: size the battery so every point of the mission keeps the diversion reserve
p.SOC_div_floor = [];      % []: keep SOC_min in a diversion; 0: the floor may be used in an emergency
% --- Battery choice and power schedule (A7) ---
p.mB_fix    = 1050;        % battery mass [kg]: [] sizes it here; 1050 keeps the PDR pack [TEAM]
p.rech_max  = true;        % true: the fuel leg recharges as much as the engine allows; false: only what is needed
p.aux_fl    = true;        % aux load on the M/G while the engine drives (climb, fuel leg)
p.aux_lo    = true;        % aux load on the battery in take-off, restarts, descent and landing
p.topup_clb = true;        % the M/G tops the battery back up in the climb [A7 power schedule]
% --- Offtakes (A7) ---
p.bleed_frac = 0;          % engine power available lost to bleed, as a fraction (we take no customer bleed)
p.f_inlet  = 0;            % installed inlet/exhaust loss, fraction of power available (0.02 = sensitivity) [EST]
p.f_ai     = 0;            % engine inlet anti-ice, fraction of P_TO (0 unless in icing) [TM]
p.f_ai_ice = 0.193;        % icing bound: 19.3% of take-off power, the largest T700 value, at every altitude [TM]
p.dD_ram   = 0.022;        % TMS ram-air drag on the electric legs, fraction of cruise drag: half of the full
                           % momentum loss of the cooling air (4.4%), i.e. a ducted, nozzled heat exchanger [EST]
p.dD_ram_F = 0.011;        % same on the fuel leg, where the heat load is about half; []: same as dD_ram [EST]
p.dT_hx    = 30;           % cooling-air temperature rise across the heat exchangers [K] [EST]
% --- Itemised hardware (A7) ---
p.m_MGunit = 125;          % H3X HPDM-1500, inverter included [kg] [H3X]
p.m_EPunit = 13.7;         % H3X HPDM-180D, inverter included [kg] [H3X]
p.m_R408   = 252;          % Dowty R408 propeller with spinner [kg] [R408]
p.D_R408   = 4.115;        % its diameter [m] [R408]
p.m_fuel_tank = 803.71;    % usable fuel the tanks hold [kg] [A7]
p.m_trap   = 8.04;         % trapped fuel [kg] [A7]
p.n_tank   = 2;            % fuel tanks [IN]
p.L_ec     = 2.1;          % engine to cockpit [m]: CT7 at 2.0 m, pilot at 4.1 m [A7]
p.m_oilexh = 10;           % engine oil and exhaust stub, not in the dry mass [kg] [EST allowance]
% --- P-S constraint powers at the design point: shaft power needed at the propellers [kW] [PS] ---
% take-off, RFP climb, initial climb, climb after engine failure, balked landing, 225 kt cruise,
% 30 deg bank at 225 kt and 12,000 ft, ceiling, objective 250 kt cruise
p.PS_need  = [815.32 1062.03 447.79 415.29 482.29 1056.89 1077.15 257.29 1421.11];
if pdr
    % PDR (24/10/2026): design point, fixed 1,334 kW M/G, 3.4 m propeller, 3 kW aux, separate inverters
    p.m_TO = 5577;  p.S = 80.84;  p.AR = 6;  p.CD0 = 0.0169;  p.P_MG_fixed = 1334;  p.cr_from_rating = true;  p.n_E = 2;
    p.ROD_dsc_ftmin = 1000;  p.dCD0_fth = 0;  p.aux_fl = false;  p.D_prop = 3.4;  p.V_tip = pi*3.4*1384/60;
    p.eta_p = 0.88;  p.P_aux = 3;  p.m_EPprop = 30;  p.L_MGc = 6.0;  p.ac_links = true;  p.mB_fix = [];
    p.rech_max = false;  p.aux_lo = false;  p.topup_clb = false;  p.dD_ram = 0;  p.dD_ram_F = [];
end
end

function printReport(r, p)
% The printout: every number the write-ups quote.
g0 = 9.80665;  LB = 0.45359237;  HP = 0.745699872;
ln = repmat('-', 1, 86);
fprintf('\nAE481 PROPULSION: GE CT7-8A6 (2035 OPTIMISTIC) + %.0f kW M/G + %d x %.0f kW BLOWN-LIFT EPs\n%s\n', ...
    r.P_MG, p.n_EP, p.P_EP, ln);
if r.pdr
    fprintf('   PDR configuration (24/10/2026)\n');
end

fprintf('\n1. MISSION AND CRUISE POWER\n');
fprintf('   Design point [PS]: MTOW %.0f kg, S %.2f m^2 (W/S %.0f N/m^2), AR %.0f, CD0 %.4f\n', ...
    p.m_TO, p.S, p.m_TO*g0/p.S, p.AR, p.CD0);
legs = sprintf('E1 %.0f + fuel leg %.0f', p.R_E, r.R_F);
if p.n_E == 2
    legs = [legs, sprintf(' + E2 %.0f', p.R_E)];
end
fprintf('   Climb %.0f + %s + descent %.0f = %.0f nmi; cruise %.0f kt at %.0f ft\n', ...
    r.R_clb, legs, r.R_dsc, r.R_clb + p.n_E*p.R_E + r.R_F + r.R_dsc, p.V_kt, p.h_cr_ft);
fprintf('   225 kt cruise, clean: C_L %.3f, L/D %.2f; propeller shaft power %.0f kW = %.0f%% of a %.0f kW M/G\n', ...
    r.CL_cr, r.LD_cr, r.P_sh_cr0, 100*r.P_sh_cr0/r.P_MG, r.P_MG);
fprintf('   With TMS ram drag (+%.1f%% electric legs, +%.1f%% fuel leg): %.0f kW on the M/G (%.0f%% of rating), %.0f kW on the fuel leg\n', ...
    100*p.dD_ram, 100*r.dDF, r.P_sh_cr, 100*r.P_sh_cr/r.P_MG, r.P_sh_crF);
fprintf('   Engine needs %.0f kW to hold 225 kt\n', r.P_gbin);
if r.P_sh_cr > r.P_MG
    fprintf('   ** WARNING: electric cruise needs %.0f kW, more than the %.0f kW M/G rating **\n', r.P_sh_cr, r.P_MG);
end

fprintf('\n2. ENGINE: GE CT7-8A6 turboshaft [TCDS]\n');
fprintf('   Ratings (SL): take-off %.0f kW (%.0f shp), max continuous %.0f kW, 30-s OEI %.0f kW\n', ...
    p.P_TO, p.P_TO/HP, p.P_MCP, p.P_30s);
fprintf('   2035 optimistic continuous %.0f kW; available at %.0f ft, 225 kt: %.0f kW (lapse %.3f)\n', ...
    r.P_cont, p.h_cr_ft, r.P_av, r.lapse);
fprintf('   Dry mass %.1f kg; %.3f m long x %.3f m wide x %.3f m high; output %.0f rpm (front drive)\n', ...
    p.m_eng, p.L_eng, p.W_eng, p.H_eng, p.N_eng);
fprintf('   SFC %.2f lb/(hp h) = %.4f kg/kWh; TSFC cruise %.3e 1/s, static %.3e 1/s\n', ...
    p.cp_lb, r.cp, r.TSFC_cr, r.TSFC_0);
fprintf('   Static thrust, ISA SL, %.1f m prop, FM %.2f: take-off %.1f kN (%.0f lbf); max (30-s) %.1f kN (%.0f lbf)\n', ...
    p.D_prop, p.FM, r.T_TO/1e3, r.T_TO/(LB*g0), r.T_max/1e3, r.T_max/(LB*g0));
fprintf('   (engine + M/G together would give %.1f kN, but that case is dropped: no take-off boost)\n', r.T_EMG/1e3);
fprintf('   Each blower %.1f kN static; all eight %.1f kN\n', r.T_EP/1e3, p.n_EP*r.T_EP/1e3);

fprintf('\n3. EFFICIENCIES [TEAM/NASA/JS]\n');
fprintf('   Battery %.3f | wiring %.3f x connectors %.3f | inverter %.3f | TMS %.3f | motor %.3f | gearbox %.3f | propeller %.3f\n', ...
    p.eta_batt, p.eta_wire, p.eta_conn, p.eta_inv, p.eta_tms, p.eta_mot, p.eta_gb, p.eta_p);
fprintf('   Terminals -> motor shaft %.4f; -> propeller shaft %.4f; battery cells -> thrust %.3f\n', r.t2s, r.t2p, r.chain);

fprintf('\n4. POWER\n');
fprintf('   Cruise M/G %.0f kW; blown-lift %.0f x %.0f = %.0f kW; total electric %.0f kW\n', ...
    r.P_MG, p.n_EP, p.P_EP, p.n_EP*p.P_EP, r.P_el_tot);
fprintf('   Engine %.0f kW; total installed %.0f kW; electric share %.3f\n', ...
    p.P_TO, p.P_TO + r.P_el_tot, r.P_el_tot/(p.P_TO + r.P_el_tot));

fprintf('\n5. BATTERY (%.0f Wh/kg, %.1f kW/kg, %.0f-100%% window) [TEAM]\n', p.e_b, p.p_b, 100*p.SOC_min);
fprintf('   Energy-sized %.0f kg, power-sized %.0f kg -> carried %.0f kg (one pack, %d sections)\n', ...
    r.mB_E, r.mB_P, r.mB, p.n_sec);
fprintf('   Max energy %.1f kWh (usable %.1f kWh); max power %.0f kW; volume %.2f m^3 at %.1f kg/L\n', ...
    r.E_nom, r.E_use, r.P_bmax, r.V_pack, p.rho_pack);
fprintf('   Loads: take-off %.0f kW (%.2fC), electric cruise %.0f kW (%.2fC), recharge %.0f kW (%.2fC)\n', ...
    r.P_TOterm + r.P_lo, (r.P_TOterm + r.P_lo)/r.E_nom, r.P_ecr, r.P_ecr/r.E_nom, r.P_chg, r.P_chg/r.E_nom);
fprintf('   Worst case, M/G at full rating + blowers + aux: %.0f kW vs %.0f kW pack limit\n', r.P_worst, r.P_bmax);
fprintf('   Climb top-up %.1f kWh in %.1f min at %.0f kW (engine has %.0f kW, climb needs %.0f kW)\n', ...
    r.E_top, r.t_top, r.P_topmax, r.P_avc, r.P_clb);
fprintf('   Per leg %.1f kWh; fuel leg puts back %.1f kWh (most it could: %.1f kWh)\n', r.E_leg, r.E_in, r.E_inmax);
for k = 1:numel(r.soc)
    fprintf('   State of charge %-22s %5.1f%%\n', r.lbl{k}, 100*r.soc(k));
end

fprintf('\n6. FUEL (cp %.4f kg/kWh; idle %.0f%% of take-off flow)\n', r.cp, 100*p.f_idle);
fprintf('   Climb power %.0f kW at %.0f kt, %.0f ft/min\n', r.P_clb, p.V_clb_kt, p.ROC_clb_ftmin);
for k = 1:numel(r.fuel_ph)
    fprintf('   %-40s %6.1f kg/h x %5.1f min = %6.1f kg\n', r.ph_name{k}, r.ph_ff(k), r.ph_t(k), r.fuel_ph(k));
end
fprintf('   Design mission fuel burn (block) %.0f kg (%.0f lb); of it %.0f kg burned to recharge\n', ...
    r.m_block, r.m_block/LB, r.m_chg);
fprintf('   IFR reserve (%.0f nmi + %.0f min at %.0f kt) %.0f kg; with 30 min it would be %.0f kg\n', p.R_alt, p.t_res, p.V_kt, r.m_res45, r.m_res30);
fprintf('   TOTAL FUEL %.0f kg (%.0f lb); tanks hold %.0f kg\n', r.m_fuel, r.m_fuel/LB, p.m_fuel_tank);

fprintf('\n7. RANGE\n');
fprintf('   Design range %.0f nmi (%.0f km) of cruise with IFR reserves (plus %.0f nmi climb and descent)\n', ...
    p.R_des, p.R_des*1.852, r.R_clb + r.R_dsc);
fprintf('   Electric: mission %d x %.0f nmi; continuous from full charge %.0f nmi at 225 kt, %.0f ft\n', ...
    p.n_E, p.R_E, r.R_el_cont, p.h_cr_ft);
fprintf('   Engine failure at 5,000 ft (ISA+18 F, MTOW %.0f kg), battery only, blowers feathered (dCD0 %.4f):\n', ...
    p.m_TO, p.dCD0_fth);
fprintf('     best range %.0f kt at L/D %.1f, %.2f kWh/nmi; the glide from %.0f ft adds %.0f nmi (from %.0f ft: %.0f nmi)\n', ...
    r.Vbr_kt, r.LDmax, r.E_per_nmi, p.h_emerg_ft, r.R_glide, p.h_cr_ft, r.R_glide12);
fprintf('     just after take-off (%.1f kWh): %.0f nmi at 225 kt, %.0f nmi at best range (no glide, no landing energy)\n', ...
    r.E_emerg, r.R_eo225, r.R_eobr);
fprintf('   Diversion range from each point of the mission (required %.0f nmi; capacity x %.2f; landing energy kept):\n', ...
    p.R_emerg_req, p.f_EOL);
for k = 1:numel(r.soc)
    flag = '';
    if r.R_div(k) < p.R_emerg_req
        flag = '   << SHORT';
    end
    fprintf('     %-22s charge %5.1f%%  %5.0f nmi%s\n', r.lbl{k}, 100*r.soc(k), r.R_div(k), flag);
end
if p.keep_res
    fprintf('   Diversion reserve %.0f kWh held throughout (included in the battery size)\n', r.E_keep);
else
    fprintf('   Diversion reserve needed %.0f kWh (%.0f%% of the pack); set keep_res = true to size the battery for it\n', ...
        r.E_res/p.f_EOL, 100*r.E_res/p.f_EOL/r.E_nom);
end

fprintf('\n8. MASSES\n');
fprintf('   (a) Parametric, NASA goals: cruise M/G motor %.1f kg + inverter %.1f kg (+%.0f%% installation = %.1f kg)\n', ...
    r.m_MG, r.m_iMG, 100*p.f_inst, (r.m_MG + r.m_iMG)*(1 + p.f_inst));
fprintf('       Blown-lift motors: %d x %.2f kg = %.1f kg; inverters %d x %.2f kg = %.1f kg\n', ...
    p.n_EP, r.m_EPm, p.n_EP*r.m_EPm, p.n_EP, r.m_iEP, p.n_EP*r.m_iEP);
fprintf('       One blown-lift propulsor (motor + inverter + %.0f%% installation + %.0f kg propeller): %.1f kg\n', ...
    100*p.f_inst, p.m_EPprop, r.m_1EP);
fprintf('       Installation %.1f | EP propellers %.0f | wiring %.1f | TMS %.1f (%.1f kW of heat)\n', ...
    r.m_inst, r.m_props, r.m_wire, r.m_tms, r.Q);
fprintf('       Electric propulsion system %.0f kg (%.0f kg without the %.0f kg battery)\n', r.m_el, r.m_el_x, r.mB);
fprintf('       Engine propulsion group, Eq. 7.21 (k %.2f): %.0f kg; combining gearbox + clutch + HS stage %.0f kg\n', ...
    p.k_pg, r.m_pg, r.m_gb);
fprintf('       Parametric total %.0f kg\n', r.m_prop);
fprintf('   (b) Itemised hardware:\n');
for k = 1:numel(r.it_mass)
    fprintf('       %-44s %8.2f kg\n', r.it_name{k}, r.it_mass(k));
end
fprintf('       %-44s %8.2f kg (%.0f lb)\n', 'ITEMISED PROPULSION GROUP', r.m_items, r.m_items/LB);
fprintf('       Engine group itemised %.1f kg vs Eq. 7.21 %.1f kg (Eq. 7.21 also covers installation)\n', r.m_grp_it, r.m_pg);

fprintf('\n9. LAYOUT AND DIMENSIONS\n');
fprintf('   Main propeller %.1f m, %d blades, %.0f rpm: tip %.0f m/s\n', p.D_prop, p.n_blade, r.N_prop, r.Vtip);
fprintf('   Tip Mach: helical %.3f in cruise (225 kt, %.0f ft); static %.3f at take-off (SL, ISA+18 F)\n', ...
    r.M_hel, p.h_cr_ft, r.M_tip0);
fprintf('   Combining gearbox ratios: engine %.1f:1, M/G %.1f:1\n', r.i_eng, r.i_MG);
fprintf('   M/G (%.0f rpm) envelope about %.2f m diameter x %.2f m long [EST]\n', p.N_MG, r.D_MGm, r.L_MGm);
fprintf('   Blower: %.2f m propeller at %.0f rpm; motor at %.0f rpm with %.0f:1 reduction, about %.2f m x %.2f m [EST]\n', ...
    p.D_EP, r.N_EPprop, r.N_EPmot, p.i_EP, r.D_EPm, r.L_EPm);
fprintf('          (direct-drive motor would be about %.2f m x %.2f m and well below %.0f kW/kg)\n', ...
    r.D_EPd, r.L_EPd, p.sp_mot);
fprintf('   Blower stations each side: %s m\n', sprintf('%.2f ', p.y_EP));

fprintf('\n10. OFFTAKES AND FEASIBILITY (aux %.0f kW normal, %.0f kW icing; anti-ice bound %.1f%% of P_TO) [A7]\n', ...
    p.P_aux, p.P_aux_ice, 100*p.f_ai_ice);
fprintf('   Customer bleed: none taken (CT7-8A6 allows %.1f%% of core airflow) [TCDS]\n', 100*p.bleed_max);
fprintf('   TMS cooling air %.2f kg/s for %.1f kW at dT %.0f K; full-loss ram drag %.0f N = %.1f%% of cruise drag (carried: %.1f%%)\n', ...
    r.mdot_hx, r.Q, p.dT_hx, r.D_ram, 100*r.D_ram/r.D_cr, 100*p.dD_ram);
fprintf('   %-46s %8s  %-24s %9s %7s %9s %7s\n', 'P-S constraint', 'need kW', 'source', 'normal', 'margin', 'icing', 'margin');
for k = 1:numel(r.chk_need)
    fprintf('   %-46s %8.0f  %-24s %9.0f %+6.0f%% %9.0f %+6.0f%%\n', r.chk_name{k}, r.chk_need(k), r.chk_src{k}, ...
        r.chk_av(k), 100*r.chk_mar(k), r.chk_avi(k), 100*r.chk_mari(k));
end
fprintf('   (icing: engine kept running, no engine-off legs; NaN = not flown in icing)\n');
fprintf('%s\n', ln);
end

function [rho, T] = atmISA(h, dT)
% ISA troposphere: density [kg/m^3] and temperature [K] at height h [m], with temperature offset dT [K]
% (pressure from the standard day, temperature raised by dT).
T0 = 288.15;  p0 = 101325;  L = 0.0065;  R = 287.05287;  g = 9.80665;
Ti = T0 - L*h;
p  = p0*(Ti/T0)^(g/(R*L));
T  = Ti + dT;
rho = p/(R*T);
end

function [D_out, L_out] = motorEnvelope(P_kW, rpm, sigma)
% Rotor volume from torque, T = 2*sigma*V_rotor, with rotor L/D = 1.
% Outer diameter 1.6 x rotor (stator + housing); length 1.5 x rotor (end windings, bearings).
T     = P_kW*1e3/(rpm*2*pi/60);
Dr    = (4*(T/(2*sigma))/pi)^(1/3);
D_out = 1.6*Dr;
L_out = 1.5*Dr;
end
