function [Results] = Weight_Predict(range_nmi, cruise_kt, altitude_ft, LD, num_people, text, electricRange_nmi)
%% ========================================================================
%  1. DESIGN INPUTS
%  ========================================================================
kgToLb = 1 / 0.45359237;
% ----- Initial weight guess -----
% 
% This guess is the mean mass of the PA-18, Kodiak 100, PC-6, P2012
% Comparison Aircraft. This is purely a starting point guess and given the
% new electric propulsion aspect, is likely to be incorrect to the final
% estimate. 
%
mTO = 3757;                     % Initial takeoff-mass guess [kg]

% ----- Payload -----
mPeople  = 86.1825*num_people;                 % Crew + passengers [kg]
mBaggage = 13.6075*num_people;                 % Baggage [kg]

% ----- Mission -----
prop = contains(text, "electric", IgnoreCase=true);
%range_nmi         = 400;        % Total design range [nmi]
%conventional aircraft cannot cruise electric
if ~prop
    electricRange_nmi = 0;     % Battery-only cruise range [nmi]
end

% ----- Reverse Mission Speed ------
reserve_kt = cruise_kt * 0.8;

% ----- Aerodynamics -----
%LD = 16;                        % Cruise L/D - advanced design assumption

% ----- Fuel propulsion -----
Cp = 0.40;                      % [lb/(hp*hr)]
                                % Low end of lecture twin-turboprop range
etaProp = 0.85;                 % Propeller efficiency
                                % Lecture twin-turboprop value
                                
% ----- Mission altitude and climb/descent -----
climb_kt      = 120;
climb_ftmin   = 1500;
descent_kt    = 150;
descent_ftmin = 1000;

% ----- Reserve mission -----
alternate_nmi = 50;             % Alternate distance [nmi]
reserve_min   = 30;             % Reserve duration [min]

% ----- Battery-only contingency -----
if prop == true
    engineOut_nmi = 20;             % Additional battery-only range [nmi]
    engineOut_kt  = 120;            % Contingency speed [kt]
else
    engineOut_nmi = 0;             % Additional battery-only range [nmi]
    engineOut_kt  = 120;            % Contingency speed [kt]
end

%% ========================================================================
%  2. ELECTRIC PROPULSION ASSUMPTIONS
%  ========================================================================
% Lecture component efficiencies
etaMotor    = 0.96;
etaInverter = 0.98;
etaCable    = 0.996;
etaBattery  = 0.96;

% Installed electric propulsion
if prop == true
    electricShaft_kW = 550;         % Combined motor shaft rating [kW]
    boost_min            = 5;       % Electric takeoff/climb boost [min]
    landing_min          = 0;       % Electric landing duration [min]
    landingPowerFraction = 0.25;    % Fraction of motor rating during landing
    auxiliary_kW = 3;               % Battery-side auxiliary load [kW]
else
    electricShaft_kW = 0;           % Combined motor shaft rating [kW]
    boost_min            = 0;       % Electric takeoff/climb boost [min]
    landing_min          = 0;       % Electric landing duration [min]
    landingPowerFraction = 0;       % Fraction of motor rating during landing
    auxiliary_kW = 0;               % Battery-side auxiliary load [kW]
end

% ----- Future battery assumptions -----
pack_Whkg  = 750;               % PACK-level specific energy [Wh/kg]
usableSOC  = 0.85;              % Usable fraction of nominal battery energy
pack_kWkg  = 2.0;               % Pack specific power [kW/kg]

% ----- Lecture electrical hardware values -----
motor_kWkg    = 10;             % Motor specific power [kW/kg]
inverter_kWkg = 16;             % Inverter specific power [kW/kg]

% Additional installation assumptions
installationFraction = 0.30;
if prop == true
    extraHardware_kg     = 5;
else
    extraHardware_kg     = 0;
end

%% ========================================================================
%  3. EMPTY-WEIGHT REGRESSION
%
%  Lecture method:
%
%       log10(WTO) = A + B*log10(WE)
%
%  Data are in pounds because the regression coefficients depend on units.
%  ========================================================================
% Comparison-aircraft data
% PA-18, Kodiak 100, PC-6, P2012
WTOdata = [1750;
           7255;
           6173;
           8113];
WEdata  = [930;
           4132;
           3075;
           5040];

% Construct lecture least-squares system:
%
%       M * [A; B] = R
M = [ones(length(WEdata),1), log10(WEdata)];
R = log10(WTOdata);

% Same least-squares solution shown in lecture,
% using "\" rather than explicit matrix inversion.
coeff = (M.' * M) \ (M.' * R);
A = coeff(1);
B = coeff(2);

% Resulting relation:
%
% log10(WTO) = A + B log10(WE)
%
% Therefore:
%
% WE = 10^[(log10(WTO) - A)/B]

%% ========================================================================
%  4. ELECTRIC PROPULSION HARDWARE MASS
%
%  Lecture values:
%       Motor    = 10 kW/kg
%       Inverter = 16 kW/kg
%  ========================================================================
etaElectrical = etaMotor * etaInverter * etaCable * etaBattery;
etaOverall = etaProp * etaElectrical;

% Motor mass
mMotor = electricShaft_kW / motor_kWkg;

% Inverter must provide the electrical power entering the motor
inverterPower_kW = electricShaft_kW / etaMotor;
mInverter = inverterPower_kW / inverter_kWkg;

% Installed electric hardware
mElectric = ...
    (mMotor + mInverter) * (1 + installationFraction) ...
    + extraHardware_kg;

%% ========================================================================
%  5. MISSION DISTANCES
%  ========================================================================
% Distance traveled during climb
climbTime_hr = (altitude_ft / climb_ftmin) / 60;
climbDistance_nmi = climb_kt * climbTime_hr;

% Distance traveled during descent
descentTime_hr = (altitude_ft / descent_ftmin) / 60;
descentDistance_nmi = descent_kt * descentTime_hr;

% Remaining mission distance flown using fuel propulsion
fuelCruise_nmi = ...
    range_nmi ...
    - climbDistance_nmi ...
    - descentDistance_nmi ...
    - electricRange_nmi;
assert(electricRange_nmi >= 0, ...
    'Electric range cannot be negative.');
assert(fuelCruise_nmi >= 0, ...
    'Fuel cruise range cannot be negative');

%% ========================================================================
%  6. FUEL FRACTION
%
%  Lecture method:
%
%  Overall mission ratio =
%
%       (W1/W0)(W2/W1)...(WN/WN-1)
%
%  Fuel fraction =
%
%       WF/W0 = 1 - Wfinal/W0
%
%  Twin-turboprop Roskam factors from lecture slide 31.
%  ========================================================================
% Twin-turboprop historical mission-segment fractions
rWarmup  = 0.990;
rTaxi    = 0.995;
rTakeoff = 0.995;
rClimb   = 0.985;
rDescent = 0.985;
rLanding = 0.995;

% -------------------------------------------------------------------------
% Propeller Breguet equation
%
% Lecture:
%
% R_nmi = 325.9 * (etaProp/Cp) * (L/D) * ln(Wi/Wf)
%
% Therefore:
%
% Wf/Wi = exp[-R*Cp/(325.9*etaProp*(L/D))]
% -------------------------------------------------------------------------
K_Breguet = 325.9; %Converts customary units to nmi

% Reserve duration represented as equivalent cruise distance.
%
% This assumes reserve flight uses the same representative cruise
% speed, L/D, Cp, and propeller efficiency.
reserveEquivalent_nmi = reserve_kt * reserve_min / 60;

% Total fuel-powered cruise-equivalent distance
fuelPoweredRange_nmi = ...
    fuelCruise_nmi ...
    + alternate_nmi ...
    + reserveEquivalent_nmi;

% Fuel-powered Breguet segment
rFuelCruise = exp( ...
    -fuelPoweredRange_nmi * Cp ...
    / (K_Breguet * etaProp * LD) );

% Complete fuel-burning mission fraction
%
% IMPORTANT:
% Non-cruise segments are included ONCE.
% The old code applied them once to the normal mission and again to
% the reserve mission.
missionWeightRatio = ...
    rWarmup ...
    * rTaxi ...
    * rTakeoff ...
    * rClimb ...
    * rFuelCruise ...
    * rDescent ...
    * rLanding;
fuelFraction = 1 - missionWeightRatio;

%% ========================================================================
%  7. FIXED-POINT TAKEOFF-WEIGHT ITERATION
%
%  Mirrors the fixed-point procedure in the lecture.
%  ========================================================================
tolerance     = 1e-6;
maxIterations = 5000;
converged = false;
for iteration = 1:maxIterations
    % --------------------------------------------------------------------
    % Empty mass from historical regression
    % ---------------------------------------------------------------------
    WTO_lb = mTO * kgToLb;
    WEconventional_lb = ...
        10^((log10(WTO_lb) - A) / B);
    mConventional = WEconventional_lb / kgToLb;
    % Hybrid empty mass:
    % conventional empty mass + separately added electric hardware
    %
    % Battery and fuel are NOT included here.
    mEmpty = mConventional + mElectric;

    % --------------------------------------------------------------------
    % Fuel mass
    % ---------------------------------------------------------------------
    mFuel = fuelFraction * mTO;

    % --------------------------------------------------------------------
    % Battery energy sizing
    %
    % Lecture electric Breguet relation:
    %
    %       R = eta * E / W * (L/D)
    %
    % Rearranged:
    %
    %       E = W*R / [eta*(L/D)]
    %
    % Aircraft mass is assumed constant during battery discharge.
    % mTO is used as a conservative estimate of electric-cruise mass.
    %
    % 1 nmi = 1852 meters
    %
    % 1 kWh = 3600000 Joules
    %
    % ---------------------------------------------------------------------
    Eflight_kWh = ...
        mTO * 9.80665 ...
        * (electricRange_nmi + engineOut_nmi) * 1852 ...
        / (LD * etaOverall * 3.6e6);

    % Electric boost energy
    Eboost_kWh = ...
        (electricShaft_kW / etaElectrical) ...
        * boost_min / 60;

    % Electric landing energy
    Elanding_kWh = ...
        (electricShaft_kW * landingPowerFraction ...
        / etaElectrical) * landing_min / 60;

    % Auxiliary energy
    electricHours = ...
        (boost_min + landing_min) / 60 ...
        + electricRange_nmi / cruise_kt ...
        + engineOut_nmi / engineOut_kt;
    Eaux_kWh = ...
        (auxiliary_kW / etaBattery) ...
        * electricHours;

    % Total required battery energy
    Erequired_kWh = ...
        Eflight_kWh ...
        + Eboost_kWh ...
        + Elanding_kWh ...
        + Eaux_kWh;

    % Battery mass constrained by energy
    mBatteryEnergy = ...
        1000 * Erequired_kWh ...
        / (pack_Whkg * usableSOC);

    % Battery mass constrained by power
    batteryChemicalPower_kW = ...
        electricShaft_kW / etaElectrical ...
        + auxiliary_kW / etaBattery;
    mBatteryPower = ...
        batteryChemicalPower_kW / pack_kWkg;

    % Battery must satisfy BOTH requirements
    mBattery = max( ...
        mBatteryEnergy, ...
        mBatteryPower );

    % --------------------------------------------------------------------
    % New gross takeoff mass
    %
    % Electric hardware is already contained in mEmpty,
    % so DO NOT add mElectric again here.
    %
    % Takeoff Weight Equation
    %       w0 = (wPeople + wBaggage + mBattery*g)/(1 - wEmpty/w0 - wFuel/w0)
    %
    % Rearrange this for ease of calculation in our iterations
    %
    %       w0(1 - wEmpty/w0 - wFuel/w0) = wPeople + wBaggage + mBattery*g
    %       w0 - wEmpty - wFuel = wPeople + wBaggage + mBattery*g
    %       m0 - mEmpty - mFuel = mPeople + mBaggage + mBattery
    %
    %       m0 = mEmpty + mFuel + mPeople + mBaggage + mBattery
    %
    % ---------------------------------------------------------------------
    mTO_new = ...
        mEmpty ...
        + mFuel ...
        + mPeople ...
        + mBaggage ...
        + mBattery;

    % Convergence check
    relativeError = ...
        abs(mTO_new - mTO) / mTO_new;
    mTO = mTO_new;

    if relativeError < tolerance
        converged = true;
        break;
    end
    assert( ...
        isfinite(mTO) && mTO > 0 && mTO < 25000, ...
        ['No weight closure within the numerical search limit. ' ...
         'Revisit the design assumptions.'] );
end
assert( ...
    converged, ...
    'Weight iteration did not converge.' );
%% ========================================================================
%  8. WEIGHTS - OVERALL OUTPUT
%
%  Column order matches B:H in the spreadsheet.
%  ========================================================================
values_kg = [ ...
    mTO, ...
    mEmpty, ...
    mFuel, ...
    mPeople, ...
    mBaggage, ...
    mElectric, ...
    mBattery ];
values_lb = values_kg * kgToLb;
Results = array2table( ...
    [values_kg; values_lb], ...
    'VariableNames', { ...
        'GrossTakeoff', ...
        'Empty', ...
        'Fuel', ...
        'CrewPassengers', ...
        'Baggage', ...
        'ElectricHardware', ...
        'Battery'}, ...
    'RowNames', {'kg','lb'} );
disp(Results);

%% ========================================================================
%  9. BOOKKEEPING CHECK
%
%  Because electric hardware is already contained inside Empty:
%
%       Gross TO = Empty + Fuel + People + Baggage + Battery
%
%  Column G is shown separately for information only.
%  ========================================================================
massCheck = ...
    mEmpty ...
    + mFuel ...
    + mPeople ...
    + mBaggage ...
    + mBattery;
assert( ...
    abs(massCheck - mTO) < 1e-3, ...
    'Final weight bookkeeping does not balance.' );
%% ========================================================================
%  10. RFP MTOW CHECK
%  ========================================================================
MTOW_limit_lb = 19000;
if prop == true
    if mTO * kgToLb <= MTOW_limit_lb
        fprintf('Electric Aircraft')
        fprintf('\nMTOW requirement satisfied: %.0f lb <= %.0f lb\n', ...
            mTO * kgToLb, MTOW_limit_lb);
    else
        fprintf('Electric Aircraft')
        fprintf('\nWARNING: MTOW requirement exceeded: %.0f lb > %.0f lb\n', ...
            mTO * kgToLb, MTOW_limit_lb);
    end
else
    fprintf('Not Electric Aircraft\n')
end
end