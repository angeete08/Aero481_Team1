function r = Weight_Predict_PS(S_m2,P_installed_kW,p)
% WEIGHT_PREDICT_PS  Parametric takeoff-weight model for the AE481 P-S plot.
%
% r = Weight_Predict_PS(S_m2,P_installed_kW,p)
%
% Inputs
%   S_m2              reference wing area [m^2]
%   P_installed_kW     total installed propulsive shaft power [kW]
%   p                  parameter structure created by
%                      AE481_PS_Environmental_Parametric.m
%
% This is intentionally a SIMPLE, anchored implementation of the Metabook
% Section 4.12 / Algorithm 2 idea: W0 = f(S,P). The current historical
% empty-weight estimate is used as the baseline; wing and thermal-propulsion
% weights are changed relative to that baseline, while electric motors,
% inverters, and the generator are explicitly resized with P.
%
% The anchor avoids extrapolating the existing empty-weight regression far
% outside its calibration range. Replace this with a full Chapter-7 component
% buildup later in preliminary design if the team has the required geometry.

    if ~(isscalar(S_m2) && isfinite(S_m2) && S_m2 > 0)
        error('S_m2 must be a positive finite scalar.');
    end
    if ~(isscalar(P_installed_kW) && isfinite(P_installed_kW) && P_installed_kW > 0)
        error('P_installed_kW must be a positive finite scalar.');
    end

    % ---------------------------------------------------------------------
    % Aerodynamics: Metabook Eq. (4.58), calibrated to the current design.
    % ---------------------------------------------------------------------
    CD0 = p.Cf_equiv*(p.Swet_rest_m2 + 2*S_m2)/S_m2;
    K   = 1/(pi*p.AR*p.e_clean);

    LD_raw     = 1/(2*sqrt(CD0*K));
    LD_raw_ref = 1/(2*sqrt(p.CD0_ref*K));
    LD         = p.LD_ref*(LD_raw/LD_raw_ref);

    % ---------------------------------------------------------------------
    % S-dependent fuel fraction: same simple mission/Breguet structure used
    % in the team's uploaded Weight_Predict code, but with L/D = L/D(S).
    % ---------------------------------------------------------------------
    fuelCruise_nmi = p.range_nmi - p.climbDistance_nmi ...
        - p.descentDistance_nmi - p.electricRange_nmi;

    if fuelCruise_nmi < 0
        error('Fuel-powered cruise range became negative.');
    end

    fuelPoweredRange_nmi = fuelCruise_nmi + p.alternate_nmi ...
        + p.reserve_kt*p.reserve_min/60;

    % Rearranged bregeut range equation
    rFuelCruise = exp(-fuelPoweredRange_nmi*p.Cp_lb_hphr / ...
        (p.K_Breguet*p.etaProp*LD));

    missionWeightRatio = p.rWarmup*p.rTaxi*p.rTakeoff*p.rClimb ...
        * rFuelCruise*p.rDescent*p.rLanding;
    fuelFraction = 1 - missionWeightRatio;

    % ---------------------------------------------------------------------
    % Parametric empty mass.
    % ---------------------------------------------------------------------
    mBaseConventional_kg = p.mEmpty_baseline_kg - p.mElectric_baseline_kg;

    mWing_kg = raymerWingMassPS(S_m2,p.mTO_baseline_kg,p);
    mWingRef_kg = raymerWingMassPS(p.S_ref_m2,p.mTO_baseline_kg,p);
    dWing_kg = mWing_kg - mWingRef_kg;

    mThermalGroup_kg = turbopropGroupMassPS(P_installed_kW,p);
    mThermalGroupRef_kg = turbopropGroupMassPS(p.P_ref_kW,p);
    dThermalGroup_kg = mThermalGroup_kg - mThermalGroupRef_kg;

    Pelectric_kW = p.electricShare*P_installed_kW;
    mElectric_kg = electricHardwareMassPS(Pelectric_kW,p);

    Pgenerator_kW = p.generatorFractionOfElectric*Pelectric_kW;
    mGenerator_kg = (Pgenerator_kW/p.generator_kWkg) ...
        *(1 + p.generatorInstallationFraction);

    mEmpty_kg = mBaseConventional_kg + dWing_kg + dThermalGroup_kg ...
        + mElectric_kg + mGenerator_kg;

    % ---------------------------------------------------------------------
    % Iterate gross takeoff mass because fuel and electric-cruise battery
    % energy depend on aircraft mass.
    % ---------------------------------------------------------------------
    mTO_kg = p.mTO_baseline_kg;
    converged = false;

    for iter = 1:p.maxWeightIterations
        mFuel_kg = fuelFraction*mTO_kg;

        etaElectrical = p.etaMotor*p.etaInverter*p.etaCable*p.etaBattery;
        etaOverall = p.etaProp*etaElectrical;

        % Rearranged bregeut range equation
        Eflight_kWh = mTO_kg*p.g0 ...
            *(p.electricRange_nmi + p.engineOut_nmi)*1852 ...
            /(LD*etaOverall*3.6e6);

        Eboost_kWh = (Pelectric_kW/etaElectrical)*p.boost_min/60;

        electricHours = p.boost_min/60 ...
            + p.electricRange_nmi/p.cruise_kt ...
            + p.engineOut_nmi/p.engineOut_kt;
        Eaux_kWh = (p.auxiliary_kW/p.etaBattery)*electricHours;

        Erequired_kWh = Eflight_kWh + Eboost_kWh + Eaux_kWh;

        mBatteryEnergy_kg = 1000*Erequired_kWh ...
            /(p.pack_Whkg*p.usableSOC);

        batteryChemicalPower_kW = Pelectric_kW/etaElectrical ...
            + p.auxiliary_kW/p.etaBattery;
        mBatteryPower_kg = batteryChemicalPower_kW/p.pack_kWkg;

        mBattery_kg = max(mBatteryEnergy_kg,mBatteryPower_kg);

        mNew_kg = mEmpty_kg + mFuel_kg + p.mPeople_kg ...
            + p.mBaggage_kg + mBattery_kg;

        relErr = abs(mNew_kg - mTO_kg)/mNew_kg;
        mTO_kg = mNew_kg;

        if relErr < p.weightTolerance
            converged = true;
            break;
        end

        if ~isfinite(mTO_kg) || mTO_kg <= 0 || mTO_kg > p.massNumericalLimit_kg
            break;
        end
    end

    if ~converged
        r = invalidResult();
        return;
    end

    % Recompute values at converged mass for clean reporting.
    mFuel_kg = fuelFraction*mTO_kg;
    etaElectrical = p.etaMotor*p.etaInverter*p.etaCable*p.etaBattery;
    etaOverall = p.etaProp*etaElectrical;
    Eflight_kWh = mTO_kg*p.g0 ...
        *(p.electricRange_nmi + p.engineOut_nmi)*1852 ...
        /(LD*etaOverall*3.6e6);
    Eboost_kWh = (Pelectric_kW/etaElectrical)*p.boost_min/60;
    electricHours = p.boost_min/60 ...
        + p.electricRange_nmi/p.cruise_kt ...
        + p.engineOut_nmi/p.engineOut_kt;
    Eaux_kWh = (p.auxiliary_kW/p.etaBattery)*electricHours;
    Erequired_kWh = Eflight_kWh + Eboost_kWh + Eaux_kWh;
    mBatteryEnergy_kg = 1000*Erequired_kWh/(p.pack_Whkg*p.usableSOC);
    batteryChemicalPower_kW = Pelectric_kW/etaElectrical ...
        + p.auxiliary_kW/p.etaBattery;
    mBatteryPower_kg = batteryChemicalPower_kW/p.pack_kWkg;
    mBattery_kg = max(mBatteryEnergy_kg,mBatteryPower_kg);

    % Nameplate pack energy implied by the converged battery mass.
    Epack_nominal_kWh = mBattery_kg*p.pack_Whkg/1000;

    r.valid = true;
    r.mTO_kg = mTO_kg;
    r.mEmpty_kg = mEmpty_kg;
    r.mFuel_kg = mFuel_kg;
    r.mBattery_kg = mBattery_kg;
    r.mElectricHardware_kg = mElectric_kg;
    r.mGenerator_kg = mGenerator_kg;
    r.mWingModel_kg = mWing_kg;
    r.mThermalGroupModel_kg = mThermalGroup_kg;
    r.fuelFraction = fuelFraction;
    r.CD0_clean = CD0;
    r.LD_cruise = LD;
    r.EbatteryRequired_kWh = Erequired_kWh;
    r.EpackNominal_kWh = Epack_nominal_kWh;
    r.Pelectric_kW = Pelectric_kW;
    r.Pgenerator_kW = Pgenerator_kW;
end

function mWing_kg = raymerWingMassPS(S_m2,mTO_kg,p)
% Metabook Eq. (7.11), Raymer transport-aircraft wing-weight correlation.
% Used only as a DIFFERENTIAL S-dependence around the baseline design.

    S_ft2 = S_m2*10.7639104167;
    Wdg_lb = mTO_kg*2.20462262185;
    Scsw_ft2 = p.controlSurfaceFraction*S_ft2;

    Wwing_lb = 0.0051*(Wdg_lb*p.Nz_ultimate)^0.557 ...
        *S_ft2^0.649*p.AR^0.5*p.tc_root^(-0.4) ...
        *(1 + p.taperRatio)^0.1*(cosd(p.sweep25_deg))^(-1) ...
        *Scsw_ft2^0.1;

    mWing_kg = Wwing_lb*0.45359237;
end

function mGroup_kg = turbopropGroupMassPS(P_installed_kW,p)
% Metabook Eqs. (7.20)-(7.21).
% The central engine must supply its propulsive share after the specified
% engine offtake is removed.

    PengineRated_kW = p.thermalShare*P_installed_kW/(1 - p.engineOfftakeFraction);
    PTO_hp = PengineRated_kW/0.745699872;

    Weng_lb = PTO_hp^0.9306 * 10^(-0.1205);
    Wpg_lb = p.kPropulsionGroup*(Weng_lb + 0.24*PTO_hp);

    mGroup_kg = Wpg_lb*0.45359237;
end

function mElectric_kg = electricHardwareMassPS(Pelectric_kW,p)
    mMotor_kg = Pelectric_kW/p.motor_kWkg;
    inverterPower_kW = Pelectric_kW/p.etaMotor;
    mInverter_kg = inverterPower_kW/p.inverter_kWkg;

    mElectric_kg = (mMotor_kg + mInverter_kg) ...
        *(1 + p.electricInstallationFraction) + p.extraElectricHardware_kg;
end

function r = invalidResult()
    r.valid = false;
    r.mTO_kg = NaN;
    r.mEmpty_kg = NaN;
    r.mFuel_kg = NaN;
    r.mBattery_kg = NaN;
    r.mElectricHardware_kg = NaN;
    r.mGenerator_kg = NaN;
    r.mWingModel_kg = NaN;
    r.mThermalGroupModel_kg = NaN;
    r.fuelFraction = NaN;
    r.CD0_clean = NaN;
    r.LD_cruise = NaN;
    r.EbatteryRequired_kWh = NaN;
    r.EpackNominal_kWh = NaN;
    r.Pelectric_kW = NaN;
    r.Pgenerator_kW = NaN;
end
