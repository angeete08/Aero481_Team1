function out = Weight_Predict(in)
%% ========================================================================
%  1. DESIGN INPUTS (Unpacked from input structure)
%  ========================================================================
kgToLb = 1 / 0.45359237;

range_nmi         = in.range_nmi;
cruise_kt         = in.cruise_kt;
altitude_ft       = in.altitude_ft;
LD                = in.LD;
mTO               = in.mTO;
num_people        = in.num_people;
is_electric       = in.is_electric;
electricRange_nmi = in.electricRange_nmi;

climb_kt      = in.climb_kt;
climb_ftmin   = in.climb_ftmin;
descent_kt    = in.descent_kt;
descent_ftmin = in.descent_ftmin;

alternate_nmi = in.alternate_nmi;
reserve_min   = in.reserve_min;

Cp      = in.Cp;
etaProp = in.etaProp;

etaMotor    = in.etaMotor;
etaInverter = in.etaInverter;
etaCable    = in.etaCable;
etaBattery  = in.etaBattery;

pack_Whkg  = in.pack_Whkg;
usableSOC  = in.usableSOC;
pack_kWkg  = in.pack_kWkg;

motor_kWkg    = in.motor_kWkg;
inverter_kWkg = in.inverter_kWkg;

installationFraction = in.installationFraction;

rWarmup  = in.rWarmup;
rTaxi    = in.rTaxi;
rTakeoff = in.rTakeoff;
rClimb   = in.rClimb;
rDescent = in.rDescent;
rLanding = in.rLanding;

% ----- Payload -----
mPeople  = 86.1825 * num_people;                 % Crew + passengers [kg]
mBaggage = 13.6075 * num_people;                 % Baggage [kg]

% ----- Mission -----
prop = logical(is_electric);
if ~prop
    electricRange_nmi = 0;     % Battery-only cruise range [nmi]
end

% ----- Reverse Mission Speed ------
reserve_kt = cruise_kt * 0.8;

% ----- Reserve mission & battery-only contingency -----
if prop == true
    engineOut_nmi        = 20;             % Additional battery-only range [nmi]
    engineOut_kt         = 120;            % Contingency speed [kt]
    electricShaft_kW     = in.electricShaft_kW;
    boost_min            = in.boost_min;
    landing_min          = in.landing_min;
    landingPowerFraction = in.landingPowerFraction;
    auxiliary_kW         = in.auxiliary_kW;
    extraHardware_kg     = 5;
else
    engineOut_nmi        = 0;
    engineOut_kt         = 120;
    electricShaft_kW     = 0;
    boost_min            = 0;
    landing_min          = 0;
    landingPowerFraction = 0;
    auxiliary_kW         = 0;
    extraHardware_kg     = 0;
end

%% ========================================================================
%  2. ELECTRIC PROPULSION ASSUMPTIONS & HARDWARE MASS
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
%  3. EMPTY-WEIGHT REGRESSION
%  ========================================================================
WTOdata = [1750;
           7255;
           6173;
           8113];
WEdata  = [930;
           4132;
           3075;
           5040];

M = [ones(length(WEdata),1), log10(WEdata)];
R = log10(WTOdata);

coeff = (M.' * M) \ (M.' * R);
A = coeff(1);
B = coeff(2);

%% ========================================================================
%  4. MISSION DISTANCES
%  ========================================================================
climbTime_hr = (altitude_ft / climb_ftmin) / 60;
climbDistance_nmi = climb_kt * climbTime_hr;

descentTime_hr = (altitude_ft / descent_ftmin) / 60;
descentDistance_nmi = descent_kt * descentTime_hr;

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
%  5. FUEL FRACTION
%  ========================================================================
K_Breguet = 325.9; % Converts customary units to nmi

reserveEquivalent_nmi = reserve_kt * reserve_min / 60;

fuelPoweredRange_nmi = ...
    fuelCruise_nmi ...
    + alternate_nmi ...
    + reserveEquivalent_nmi;

rFuelCruise = exp( ...
    -fuelPoweredRange_nmi * Cp ...
    / (K_Breguet * etaProp * LD) );

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
%  6. FIXED-POINT TAKEOFF-WEIGHT ITERATION
%  ========================================================================
tolerance     = 1e-6;
maxIterations = 5000;
converged     = false;

for iteration = 1:maxIterations
    WTO_lb = mTO * kgToLb;
    WEconventional_lb = ...
        10^((log10(WTO_lb) - A) / B);
    mConventional = WEconventional_lb / kgToLb;
    
    mEmpty = mConventional + mElectric;
    mFuel = fuelFraction * mTO;

    Eflight_kWh = ...
        mTO * 9.80665 ...
        * (electricRange_nmi + engineOut_nmi) * 1852 ...
        / (LD * etaOverall * 3.6e6);

    Eboost_kWh = ...
        (electricShaft_kW / etaElectrical) ...
        * boost_min / 60;

    Elanding_kWh = ...
        (electricShaft_kW * landingPowerFraction ...
        / etaElectrical) * landing_min / 60;

    electricHours = ...
        (boost_min + landing_min) / 60 ...
        + electricRange_nmi / cruise_kt ...
        + engineOut_nmi / engineOut_kt;
        
    Eaux_kWh = ...
        (auxiliary_kW / etaBattery) ...
        * electricHours;

    Erequired_kWh = ...
        Eflight_kWh ...
        + Eboost_kWh ...
        + Elanding_kWh ...
        + Eaux_kWh;

    mBatteryEnergy = ...
        1000 * Erequired_kWh ...
        / (pack_Whkg * usableSOC);

    batteryChemicalPower_kW = ...
        electricShaft_kW / etaElectrical ...
        + auxiliary_kW / etaBattery;
        
    mBatteryPower = ...
        batteryChemicalPower_kW / pack_kWkg;

    mBattery = max( ...
        mBatteryEnergy, ...
        mBatteryPower );

    mTO_new = ...
        mEmpty ...
        + mFuel ...
        + mPeople ...
        + mBaggage ...
        + mBattery;

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
%  7. WEIGHTS - NUMERIC OUTPUT VECTOR
%  ========================================================================
values_kg = [ ...
    mTO, ...
    mEmpty, ...
    mFuel, ...
    mPeople, ...
    mBaggage, ...
    mElectric, ...
    mBattery ];

%% ========================================================================
%  8. BOOKKEEPING CHECK
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

% Output the clean numeric vector to Simulink
out = values_kg;
end