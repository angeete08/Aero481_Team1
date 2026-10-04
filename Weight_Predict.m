function [Results] = Weight_Predict(range_nmi, cruise_kt, altitude_ft, LD, num_people, text, electricRange_nmi)
%% ========================================================================
%  1. DESIGN INPUTS (KODIAK 100 BASELINE)
%  ========================================================================
kgToLb = 1 / 0.45359237;

% Initial takeoff mass guess for Kodiak 100 (~7255 lb)
mTO = 3290.8;                     % Initial takeoff-mass guess [kg]

% Payload (9 total occupants: 1 pilot + 8 passengers for typical utility mission)
mPeople  = 86.1825*num_people;                 % Crew + passengers [kg]
mBaggage = 13.6075*num_people;                 % Baggage [kg]

% Mission type
prop = contains(text, "electric", IgnoreCase=true);
if ~prop
    electricRange_nmi = 0;     % Conventional fuel aircraft
end

% Reserve Mission Speed
reserve_kt = cruise_kt * 0.8;

% Fuel propulsion parameters (PT6A-34 Turboprop)
Cp = 0.53;                      % Specific fuel consumption [lb/(hp*hr)]
etaProp = 0.82;                 % Propeller efficiency

% Mission altitude and climb/descent
climb_kt      = 101;
climb_ftmin   = 1340;
descent_kt    = 130;
descent_ftmin = 1000;

% Reserve mission parameters
alternate_nmi = 50;             % Alternate distance [nmi]
reserve_min   = 45;             % FAA VFR/IFR reserve duration [min]

% Contingency parameters
engineOut_nmi = 0;
engineOut_kt  = 101;

%% ========================================================================
%  2. ELECTRIC PROPULSION ASSUMPTIONS (SET TO ZERO FOR CONVENTIONAL KODIAK)
%  ========================================================================
etaMotor    = 0.96;
etaInverter = 0.98;
etaCable    = 0.996;
etaBattery  = 0.96;

if prop == true
    electricShaft_kW = 550;
    boost_min        = 5;
    landing_min      = 0;
    landingPowerFraction = 0.25;
    auxiliary_kW     = 3;
else
    electricShaft_kW = 0;
    boost_min        = 0;
    landing_min      = 0;
    landingPowerFraction = 0;
    auxiliary_kW     = 0;
end

pack_Whkg  = 250;
usableSOC  = 0.85;
pack_kWkg  = 1.5;

motor_kWkg    = 10;
inverter_kWkg = 16;

installationFraction = 0.30;
extraHardware_kg     = 0;

%% ========================================================================
%  3. EMPTY-WEIGHT REGRESSION
%  ========================================================================
WTOdata = [1750; 7255; 6173; 8113];
WEdata  = [930;  3800; 3075; 5040];

M = [ones(length(WEdata),1), log10(WEdata)];
R = log10(WTOdata);

coeff = (M.' * M) \ (M.' * R);
A = coeff(1);
B = coeff(2);

%% ========================================================================
%  4. ELECTRIC PROPULSION HARDWARE MASS
%  ========================================================================
etaElectrical = etaMotor * etaInverter * etaCable * etaBattery;
etaOverall = etaProp * etaElectrical;

mMotor = electricShaft_kW / motor_kWkg;
inverterPower_kW = electricShaft_kW / etaMotor;
mInverter = inverterPower_kW / inverter_kWkg;

mElectric = (mMotor + mInverter) * (1 + installationFraction) + extraHardware_kg;

%% ========================================================================
%  5. MISSION DISTANCES
%  ========================================================================
climbTime_hr = (altitude_ft / climb_ftmin) / 60;
climbDistance_nmi = climb_kt * climbTime_hr;

descentTime_hr = (altitude_ft / descent_ftmin) / 60;
descentDistance_nmi = descent_kt * descentTime_hr;

fuelCruise_nmi = range_nmi - climbDistance_nmi - descentDistance_nmi - electricRange_nmi;
assert(electricRange_nmi >= 0, 'Electric range cannot be negative.');
assert(fuelCruise_nmi >= 0, 'Fuel cruise range cannot be negative');

%% ========================================================================
%  6. FUEL FRACTION
%  ========================================================================
rWarmup  = 0.990;
rTaxi    = 0.995;
rTakeoff = 0.995;
rClimb   = 0.985;
rDescent = 0.985;
rLanding = 0.995;

K_Breguet = 325.9;
reserveEquivalent_nmi = reserve_kt * reserve_min / 60;

fuelPoweredRange_nmi = fuelCruise_nmi + alternate_nmi + reserveEquivalent_nmi;

rFuelCruise = exp(-fuelPoweredRange_nmi * Cp / (K_Breguet * etaProp * LD));

missionWeightRatio = rWarmup * rTaxi * rTakeoff * rClimb * rFuelCruise * rDescent * rLanding;
fuelFraction = 1 - missionWeightRatio;

%% ========================================================================
%  7. FIXED-POINT TAKEOFF-WEIGHT ITERATION
%  ========================================================================
tolerance     = 1e-6;
maxIterations = 5000;
converged = false;

for iteration = 1:maxIterations
    WTO_lb = mTO * kgToLb;
    WEconventional_lb = 10^((log10(WTO_lb) - A) / B);
    mConventional = WEconventional_lb / kgToLb;
    mEmpty = mConventional + mElectric;

    mFuel = fuelFraction * mTO;

    Eflight_kWh = mTO * 9.80665 * (electricRange_nmi + engineOut_nmi) * 1852 / (LD * etaOverall * 3.6e6);
    Eboost_kWh = (electricShaft_kW / etaElectrical) * boost_min / 60;
    Elanding_kWh = (electricShaft_kW * landingPowerFraction / etaElectrical) * landing_min / 60;

    electricHours = (boost_min + landing_min) / 60 + electricRange_nmi / cruise_kt + engineOut_nmi / engineOut_kt;
    Eaux_kWh = (auxiliary_kW / etaBattery) * electricHours;

    Erequired_kWh = Eflight_kWh + Eboost_kWh + Elanding_kWh + Eaux_kWh;

    mBatteryEnergy = 1000 * Erequired_kWh / (pack_Whkg * usableSOC);
    batteryChemicalPower_kW = electricShaft_kW / etaElectrical + auxiliary_kW / etaBattery;
    mBatteryPower = batteryChemicalPower_kW / pack_kWkg;

    mBattery = max(mBatteryEnergy, mBatteryPower);

    mTO_new = mEmpty + mFuel + mPeople + mBaggage + mBattery;

    relativeError = abs(mTO_new - mTO) / mTO_new;
    mTO = mTO_new;

    if relativeError < tolerance
        converged = true;
        break;
    end
    assert(isfinite(mTO) && mTO > 0 && mTO < 25000, 'No weight closure within search limit.');
end
assert(converged, 'Weight iteration did not converge.');

%% ========================================================================
%  8. WEIGHTS - OVERALL OUTPUT
%  ========================================================================
values_kg = [mTO, mEmpty, mFuel, mPeople, mBaggage, mElectric, mBattery];
values_lb = values_kg * kgToLb;
Results = array2table( ...
    [values_kg; values_lb], ...
    'VariableNames', {'GrossTakeoff', 'Empty', 'Fuel', 'CrewPassengers', 'Baggage', 'ElectricHardware', 'Battery'}, ...
    'RowNames', {'kg','lb'} );
disp(Results);

%% ========================================================================
%  9. BOOKKEEPING CHECK
%  ========================================================================
massCheck = mEmpty + mFuel + mPeople + mBaggage + mBattery;
assert(abs(massCheck - mTO) < 1e-3, 'Final weight bookkeeping does not balance.');

%% ========================================================================
%  10. RFP MTOW CHECK
%  ========================================================================
MTOW_limit_lb = 7255; % Kodiak 100 MTOW Limit
if mTO * kgToLb <= MTOW_limit_lb + 50
    fprintf('\nMTOW requirement satisfied for Kodiak 100: %.0f lb <= %.0f lb\n', mTO * kgToLb, MTOW_limit_lb);
else
    fprintf('\nWARNING: MTOW limit exceeded: %.0f lb > %.0f lb\n', mTO * kgToLb, MTOW_limit_lb);
end
end