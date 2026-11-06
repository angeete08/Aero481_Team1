function values_kg = Hybrid_Aircraft_Refined_Weight(in, TailData, S_m2, P_kW)
%% ========================================================================
%  HYBRID_AIRCRAFT_REFINED_WEIGHT
%  Raymer Chapter 15 + Hybrid Propulsion Weight Breakdown Function
%  Takes pipeline inputs, TailData, and optimized design point (S_m2, P_kW).
%  ========================================================================

    p.exampleInputs = false;        
    p.initialMass_kg = in.mTO; 
    p.maximumIterationMass_kg = 50000; 
    p.tolerance_kg = 0.01;
    p.maximumIterations = 300;
    p.relaxation = 0.5;

    % Store parent input structure reference for local energy/efficiency lookups
    p.parentIn = in;

    % Occupants and useful load from pipeline
    p.passengers = in.num_people - 1; % 1 crew, rest passengers
    p.crew = in.crew;
    p.personMass_kg = in.personMass_kg;          
    p.baggagePerPassenger_kg = in.baggagePerPassenger_kg;
    p.crewBaggage_kg = in.crewBaggage_kg;         
    p.cargo_kg = 0;
    p.otherUsefulLoad_kg = 0;

    % Wing Geometry from pipeline optimization
    p.wing.S_m2 = S_m2; 
    p.wing.AR = in.AR; 
    p.wing.taper = in.taperRatio;
    p.wing.tc = in.tc_root; 
    p.wing.sweep_deg = in.sweep25_deg;

    % Tail Data from TailSizing: [Sh, b_h, c_h, Sv, h_v, c_v]
    p.ht.S_m2 = TailData(1); 
    p.ht.AR = (TailData(2)^2) / TailData(1); 
    p.ht.taper = in.ht.taper;
    p.ht.tc = in.ht.tc; 
    p.ht.sweep_deg = in.ht.sweep_deg;

    p.vt.S_m2 = TailData(4); 
    p.vt.AR = (TailData(5)^2) / TailData(4); 
    p.vt.taper = in.vt.taper;
    p.vt.tc = in.vt.tc; 
    p.vt.sweep_deg = in.vt.sweep_deg;

    p.tailHeightRatio = in.tailHeightRatio;        
    p.fuselageWettedArea_m2 = in.fuselageWettedArea_m2;
    p.fuselageStructuralLength_m = in.fuselageStructuralLength_m;
    p.fuselageDepth_m = in.fuselageDepth_m;
    p.tailArm_m = in.tailArm_m;            
    p.pressurizedVolume_m3 = in.pressurizedVolume_m3;
    p.cabinDifferentialPressure_Pa = in.cabinDifferentialPressure_Pa;
    p.mainGearLength_m = in.mainGearLength_m;     
    p.noseGearLength_m = in.noseGearLength_m;
    p.retractableGear = in.retractableGear;

    % Loads and aerodynamic state
    p.limitFlightLoadFactor = in.limitFlightLoadFactor; 
    p.ultimateLandingLoadFactor = in.ultimateLandingLoadFactor;
    p.flightDesignMassFraction = in.flightDesignMassFraction; 
    p.landingDesignMassFraction = in.landingDesignMassFraction; 
    p.cruiseSpeed_mps = in.cruise_kt * 0.514444;
    p.cruiseDensity_kgm3 = in.cruiseDensity_kgm3;
    p.maximumMach = in.maximumMach;

    p.factor.wing = in.factor.wing; 
    p.factor.ht = in.factor.ht; 
    p.factor.vt = in.factor.vt;
    p.factor.fuselage = in.factor.fuselage; 
    p.factor.gear = in.factor.gear;
    p.factor.flightControls = in.factor.flightControls;
    p.growthFraction = in.growthFraction;      
    p.unidentifiedFraction = in.unidentifiedFraction;   

    p.fuelDensity_kgm3 = in.fuelDensity_kgm3;
    p.fuelTankCount = in.fuelTankCount;
    p.integralTankVolumeFraction = in.integralTankVolumeFraction; 
    p.wingFuelFraction = in.wingFuelFraction;       
    p.trappedFuel_kg = in.trappedFuel_kg;         

    % Mission Distances & Times Aligned with Pipeline Parameters
    climbTime_hr      = (in.altitude_ft / in.climb_ftmin) / 60;
    descentTime_hr    = (in.altitude_ft / in.descent_ftmin) / 60;
    fuelCruise_nmi    = in.range_nmi - (in.climb_kt * climbTime_hr) - (descentTime_hr * in.descent_kt) - in.electricRange_nmi;
    cruiseTime_hr     = max(0.1, fuelCruise_nmi / in.cruise_kt);

    thermalShaftPower = P_kW * in.thermalShare;
    electricTerminalPower = P_kW * in.electricShare;

    % Align BSFC with pipeline Cp (convert lb/(hp*hr) to kg/(kW*hr))
    p.turbineBSFC_kgkWh = in.Cp * 0.608277;   

    p.mission = [ ...
      0.05, thermalShaftPower, electricTerminalPower, 0; ...                % Takeoff
      climbTime_hr, thermalShaftPower * 0.9, electricTerminalPower * 0.5, 0; ... % Climb
      cruiseTime_hr, thermalShaftPower * 0.85, electricTerminalPower * 0.3, 0; ...% Cruise (Fuel-powered)
      descentTime_hr, thermalShaftPower * 0.3, 0, 0; ...                    % Descent
      0.75, thermalShaftPower * 0.6, 0, 1];                                 % Reserve

    p.fuelContingencyFraction = in.fuelContingencyFraction; 
    p.energyReferenceMass_kg = in.energyReferenceMass_kg;
    p.missionPowerMassExponent = in.missionPowerMassExponent; 

    p.battery.packSpecificEnergy_Whkg = in.pack_Whkg;
    p.battery.usableEnergyFraction = in.usableSOC; 
    p.battery.packSpecificPower_kWkg = in.pack_kWkg;
    p.battery.minimumInstalledCapacity_kWh = in.battery.minimumInstalledCapacity_kWh;

    p.turbineCount = in.turbineCount;
    p.turbineMassEach_kg = in.turbineMassEach_kg;   
    p.turbineRatedShaftPowerEach_kW = thermalShaftPower;
    p.generatorCount = in.generatorCount;        
    p.generatorMassEach_kg = in.generatorMassEach_kg;

    motorPowerEach = (electricTerminalPower * in.electricShare) / in.numDistributedMotors;
    p.driveBanks = [in.numDistributedMotors, motorPowerEach, in.motor_kWkg, in.inverter_kWkg, 12, 0];

    p.extraNames = {'Nacelles and fairings'; 'Propulsion mounts'; ...
     'Distributed-lift structural increment'; 'Extra flap actuation'; ...
     'Engine accessories, exhaust and starter'; 'HV cables and protection'; ...
     'External thermal system including coolant'; 'Other installed equipment'};
    p.extraGroups = {'Structure'; 'Structure'; 'Structure'; 'Equipment'; ...
     'Propulsion'; 'Propulsion'; 'Propulsion'; 'Equipment'};
    p.extraMass_kg = [40; 30; 35; 15; 35; 45; 65; 0];
    p.extraX_m = [3.2; 3.2; 3.2; 3.5; 2; 4; 4; 4];

    p.uninstalledAvionics_kg      = in.uninstalledAvionics_kg;
    p.hydraulicsKh                = in.hydraulicsKh;           
    p.hydraulicsMach              = in.hydraulicsMach;       
    p.useRaymerElectrical         = in.useRaymerElectrical;
    p.conventionalElectrical_kg   = in.conventionalElectrical_kg; 
    p.useRaymerEnvironmental      = in.useRaymerEnvironmental;
    p.environmentalAndAntiIce_kg  = in.environmentalAndAntiIce_kg; 
    p.useRaymerFurnishings        = in.useRaymerFurnishings;
    p.furnishings_kg              = in.furnishings_kg;      

    p.x.wing                      = in.x.wing; 
    p.x.ht                        = in.x.ht; 
    p.x.vt                        = in.x.vt; 
    p.x.fuselage                  = in.x.fuselage;
    p.x.mainGear                  = in.x.mainGear; 
    p.x.noseGear                  = in.x.noseGear; 
    p.x.turbine                   = in.x.turbine; 
    p.x.generator                 = in.x.generator;
    p.x.drives                    = in.x.drives; 
    p.x.battery                   = in.x.battery; 
    p.x.fuel                      = in.x.fuel; 
    p.x.controls                  = in.x.controls;
    p.x.systems                   = in.x.systems; 
    p.x.furnishings               = in.x.furnishings; 
    p.x.crew                      = in.x.crew;
    p.x.passengers                = in.x.passengers; 
    p.x.baggage                   = in.x.baggage; 
    p.x.cargo                     = in.x.cargo;
    p.x.otherUseful               = in.x.otherUseful;

    p.wingLEMAC_m                 = in.wingLEMAC_m;
    p.wingMAC_m = S_m2 / sqrt(in.AR * S_m2);

    validateInputs(p);
    
    mass_kg = p.initialMass_kg;
    converged = false;
    prev_residual = inf;
    
    for iteration = 1:p.maximumIterations
        r = evaluateWeight(mass_kg, p);
        residual = r.total_kg - mass_kg;
        
        if abs(residual) <= p.tolerance_kg
            converged = true; 
            break;
        end
        
        % Adaptive relaxation to prevent oscillation and infinite looping
        if iteration > 1 && sign(residual) ~= sign(prev_residual)
            p.relaxation = p.relaxation * 0.5; 
        end
        
        mass_kg = mass_kg + p.relaxation * residual;
        prev_residual = residual;
        
        if ~isfinite(mass_kg) || mass_kg <= 0 || mass_kg > p.maximumIterationMass_kg
            error('Weight iteration diverged or exceeded numerical mass guard. Review inputs.');
        end
    end
    
    if ~converged
        warning('Weight iteration did not meet strict tolerance in %d iterations. Proceeding with closest closure.', p.maximumIterations);
    end

    % Extract vector components matching preliminary Weight_Predict format
    mTO      = r.total_kg;
    mEmpty   = r.empty_kg;
    mFuel    = r.usableFuel_kg;
    mPeople  = (p.passengers + p.crew) * p.personMass_kg;
    mBaggage = p.passengers * p.baggagePerPassenger_kg + p.crewBaggage_kg;
    mElectric= sum(r.mass(strcmp(r.group, 'Propulsion') & ...
               ismember(r.name, {'Electric motors','Inverters','Propellers','Gearboxes'})));
    mBattery = r.batteryMass_kg;

    values_kg = [ ...
        mTO, ...
        mEmpty, ...
        mFuel, ...
        mPeople, ...
        mBaggage, ...
        mElectric, ...
        mBattery ];
end

%% LOCAL FUNCTIONS ---------------------------------------------------------
function r=evaluateWeight(m0,p)
    kg2lb=2.20462262185; m2ft=3.28083989501;
    m2ft2=m2ft^2; pa2psf=0.020885434273;
    scale=(m0/p.energyReferenceMass_kg)^p.missionPowerMassExponent;
    t=p.mission(:,1); pt=p.mission(:,2)*scale; pb=p.mission(:,3)*scale;
    bsfc=p.turbineBSFC_kgkWh(:);
    if isscalar(bsfc), bsfc=repmat(bsfc,size(t)); end
    fuelBySegment=t.*pt.*bsfc;
    fuel=sum(fuelBySegment)*(1+p.fuelContingencyFraction);
    
    % --- ENERGY FROM MISSION SEGMENTS ---
    energy_mission = sum(t.*pb);

    % --- PURE ELECTRIC CRUISE ENERGY (200 nmi electric range requirement) ---
    parentIn = p.parentIn;
    etaElectrical = parentIn.etaMotor * parentIn.etaInverter * parentIn.etaCable * parentIn.etaBattery;
    etaOverall    = parentIn.etaProp * etaElectrical;
    
    if isfield(parentIn, 'LD_ref') && ~isempty(parentIn.LD)
        cruiseLD = parentIn.LD;
    else
        cruiseLD = 12.0; 
    end

    W_N = m0 * 9.80665;
    electricRange_m = parentIn.electricRange_nmi * 1852;
    engineOutRange_m = 20 * 1852; % 20 nmi contingency
    
    E_electric_flight_kWh = W_N * (electricRange_m + engineOutRange_m) / (cruiseLD * etaOverall * 3.6e6);
    totalEnergy_kWh = energy_mission + E_electric_flight_kWh;

    energyCapacity = max(totalEnergy_kWh / p.battery.usableEnergyFraction, ...
        p.battery.minimumInstalledCapacity_kWh);
    batEnergy = energyCapacity * 1000 / p.battery.packSpecificEnergy_Whkg;
    batPower  = max(pb) / p.battery.packSpecificPower_kWkg;
    battery   = max(batEnergy, batPower);

    Wdg=m0*kg2lb*p.flightDesignMassFraction;
    Wl=m0*kg2lb*p.landingDesignMassFraction;
    Nz=1.5*p.limitFlightLoadFactor; Nl=p.ultimateLandingLoadFactor;
    q=0.5*p.cruiseDensity_kgm3*p.cruiseSpeed_mps^2*pa2psf;
    Sw=p.wing.S_m2*m2ft2; Sh=p.ht.S_m2*m2ft2; Sv=p.vt.S_m2*m2ft2;
    Sf=p.fuselageWettedArea_m2*m2ft2;
    L=p.fuselageStructuralLength_m*m2ft; D=p.fuselageDepth_m*m2ft;
    Lt=p.tailArm_m*m2ft; Bw=sqrt(p.wing.AR*Sw);
    cw=cosd(p.wing.sweep_deg); ch=cosd(p.ht.sweep_deg); cv=cosd(p.vt.sweep_deg);
    Wfw=(fuel+p.trappedFuel_kg)*p.wingFuelFraction*kg2lb;
    fuelFactor=1; if Wfw>0, fuelFactor=Wfw^0.0035; end
    
    wing=0.036*Sw^0.758*fuelFactor*(p.wing.AR/cw^2)^0.6*q^0.006 ...
     *p.wing.taper^0.04*(100*p.wing.tc/cw)^(-0.3)*(Nz*Wdg)^0.45;
    ht=0.016*(Nz*Wdg)^0.414*q^0.168*Sh^0.896*(100*p.ht.tc/ch)^(-0.12) ...
     *(p.ht.AR/ch^2)^0.043*p.ht.taper^(-0.02);
    vt=0.073*(1+0.2*p.tailHeightRatio)*(Nz*Wdg)^0.376*q^0.122*Sv^0.873 ...
     *(100*p.vt.tc/cv)^(-0.49)*(p.vt.AR/cv^2)^0.357*max(p.vt.taper,0.2)^0.039;
    Wpress=11.9*(p.pressurizedVolume_m3*m2ft^3* ...
        p.cabinDifferentialPressure_Pa/6894.757293)^0.271;
    fuselage=0.052*Sf^1.086*(Nz*Wdg)^0.177*Lt^(-0.051)*(L/D)^(-0.072)*q^0.241+Wpress;
    main=0.095*(Nl*Wl)^0.768*(p.mainGearLength_m*m2ft*12/12)^0.409;
    nose=0.125*(Nl*Wl)^0.566*(p.noseGearLength_m*m2ft*12/12)^0.845;
    if ~p.retractableGear
        gearBefore=main+nose; gearAfter=gearBefore-0.014*m0*kg2lb;
        main=main*gearAfter/gearBefore; nose=nose*gearAfter/gearBefore;
    end
    Vt=(fuel+p.trappedFuel_kg)/p.fuelDensity_kgm3/0.003785411784;
    fuelSystem=2.49*Vt^0.726*(1/(1+p.integralTankVolumeFraction))^0.363 ...
     *p.fuelTankCount^0.242*p.turbineCount^0.157;
    controls=0.053*L^1.536*Bw^0.371*(Nz*Wdg*1e-4)^0.80;
    hydraulics=p.hydraulicsKh*Wdg^0.8*p.hydraulicsMach^0.5;
    avionics=2.117*(p.uninstalledAvionics_kg*kg2lb)^0.933;
    electrical=p.conventionalElectrical_kg*kg2lb;
    if p.useRaymerElectrical, electrical=12.57*(fuelSystem+avionics)^0.51; end
    environmental=p.environmentalAndAntiIce_kg*kg2lb;
    if p.useRaymerEnvironmental
     environmental=0.265*Wdg^0.52*(p.passengers+p.crew)^0.68*avionics^0.17*p.maximumMach^0.08;
    end
    furnishings=p.furnishings_kg*kg2lb;
    if p.useRaymerFurnishings
     furnishings=0.0582*Wdg-65;
    end
    b=p.driveBanks;
    motors=sum(b(:,1).*b(:,2)./b(:,3));
    inverters=sum(b(:,1).*b(:,2)./b(:,4));
    props=sum(b(:,1).*b(:,5)); gearboxes=sum(b(:,1).*b(:,6));
    
    r.name={}; r.group={}; r.mass=[]; r.x=[];
    r=add(r,'Wing','Structure',wing/kg2lb*p.factor.wing,p.x.wing);
    r=add(r,'Horizontal tail','Structure',ht/kg2lb*p.factor.ht,p.x.ht);
    r=add(r,'Vertical tail','Structure',vt/kg2lb*p.factor.vt,p.x.vt);
    r=add(r,'Fuselage','Structure',fuselage/kg2lb*p.factor.fuselage,p.x.fuselage);
    r=add(r,'Main landing gear','Structure',main/kg2lb*p.factor.gear,p.x.mainGear);
    r=add(r,'Nose landing gear','Structure',nose/kg2lb*p.factor.gear,p.x.noseGear);
    r=add(r,'Turbine engines (bare)','Propulsion',p.turbineCount*p.turbineMassEach_kg,p.x.turbine);
    r=add(r,'Generators','Propulsion',p.generatorCount*p.generatorMassEach_kg,p.x.generator);
    r=add(r,'Electric motors','Propulsion',motors,p.x.drives);
    r=add(r,'Inverters','Propulsion',inverters,p.x.drives);
    r=add(r,'Propellers','Propulsion',props,p.x.drives);
    r=add(r,'Gearboxes','Propulsion',gearboxes,p.x.drives);
    r=add(r,'Battery pack','Propulsion',battery,p.x.battery);
    r=add(r,'Fuel system','Propulsion',fuelSystem/kg2lb,p.x.fuel);
    r=add(r,'Trapped fuel','Propulsion',p.trappedFuel_kg,p.x.fuel);
    r=add(r,'Flight controls','Equipment',controls/kg2lb*p.factor.flightControls,p.x.controls);
    r=add(r,'Hydraulics','Equipment',hydraulics/kg2lb,p.x.systems);
    r=add(r,'Conventional electrical','Equipment',electrical/kg2lb,p.x.systems);
    r=add(r,'Installed avionics','Equipment',avionics/kg2lb,p.x.systems);
    r=add(r,'Cabin environmental and airframe anti-ice','Equipment',environmental/kg2lb,p.x.systems);
    r=add(r,'Furnishings including seats','Equipment',furnishings/kg2lb,p.x.furnishings);
    
    for i=1:numel(p.extraMass_kg)
        r=add(r,p.extraNames{i},p.extraGroups{i},p.extraMass_kg(i),p.extraX_m(i));
    end
    
    r.baseEmpty_kg=sum(r.mass);
    baseCG=sum(r.mass.*r.x)/r.baseEmpty_kg;
    r=add(r,'Weight growth allowance','Allowance',r.baseEmpty_kg*p.growthFraction,baseCG);
    r=add(r,'Unidentified-items allowance','Allowance',r.baseEmpty_kg*p.unidentifiedFraction,baseCG);
    r.empty_kg=sum(r.mass);
    
    crew = p.crew * p.personMass_kg;
    passengers = p.passengers * p.personMass_kg;
    baggage = p.passengers * p.baggagePerPassenger_kg + p.crewBaggage_kg;
    
    r=add(r,'Crew','Useful load',crew,p.x.crew);
    r=add(r,'Passengers','Useful load',passengers,p.x.passengers);
    r=add(r,'Baggage','Useful load',baggage,p.x.baggage);
    r=add(r,'Cargo','Useful load',p.cargo_kg,p.x.cargo);
    r=add(r,'Other useful load','Useful load',p.otherUsefulLoad_kg,p.x.otherUseful);
    r=add(r,'Usable mission fuel incl reserve','Useful load',fuel,p.x.fuel);
    
    r.total_kg=sum(r.mass);
    r.batteryCapacity_kWh=energyCapacity;
    r.batteryMass_kg=battery;
    r.usableFuel_kg=fuel;
end

function r=add(r,name,group,mass,x)
    assert(isfinite(mass)&&mass>=0,'Invalid mass for component: %s',name);
    r.name{end+1,1}=name; r.group{end+1,1}=group;
    r.mass(end+1,1)=mass; r.x(end+1,1)=x;
end

function validateInputs(p)
    assert(p.initialMass_kg>0 && p.energyReferenceMass_kg>0);
    assert(p.relaxation>0 && p.relaxation<=1 && p.tolerance_kg>0);
    assert(p.maximumIterations>=1 && p.maximumIterations==floor(p.maximumIterations));
    assert(p.passengers>=0 && p.crew>=1 && all(mod([p.passengers,p.crew],1)==0));
    assert(p.personMass_kg>0 && p.baggagePerPassenger_kg>=0 && p.crewBaggage_kg>=0);
    assert(p.limitFlightLoadFactor>0 && p.ultimateLandingLoadFactor>0);
    assert(p.flightDesignMassFraction>0 && p.flightDesignMassFraction<=1);
    assert(p.landingDesignMassFraction>0 && p.landingDesignMassFraction<=1);
    assert(p.cruiseDensity_kgm3>0 && p.cruiseSpeed_mps>0 && p.maximumMach>0);
    assert(p.wingMAC_m>0 && p.fuselageWettedArea_m2>0 && p.tailArm_m>0);
    assert(p.fuselageStructuralLength_m>0 && p.fuselageDepth_m>0);
    assert(p.mainGearLength_m>0 && p.noseGearLength_m>0);
    assert(p.pressurizedVolume_m3>=0 && p.cabinDifferentialPressure_Pa>=0);
    assert(p.tailHeightRatio>=0 && p.tailHeightRatio<=1);
    for g={p.wing,p.ht,p.vt}
     a=g{1}; assert(a.S_m2>0 && a.AR>0 && a.tc>0 && a.tc<1);
     assert(a.taper>=0 && a.taper<=1 && abs(a.sweep_deg)<90);
    end
    assert(size(p.mission,2)==4 && ~isempty(p.mission));
    assert(all(isfinite(p.mission(:))) && all(all(p.mission(:,1:3)>=0)));
    assert(all(ismember(p.mission(:,4),[0,1])));
    assert(all(p.turbineBSFC_kgkWh(:)>0) && ...
     (isscalar(p.turbineBSFC_kgkWh)||numel(p.turbineBSFC_kgkWh)==size(p.mission,1)));
    assert(p.battery.usableEnergyFraction>0 && p.battery.usableEnergyFraction<=1);
    assert(p.battery.packSpecificEnergy_Whkg>0 && p.battery.packSpecificPower_kWkg>0);
    assert(p.battery.minimumInstalledCapacity_kWh>=0);
    assert(p.turbineCount>=1 && mod(p.turbineCount,1)==0 && p.turbineMassEach_kg>0);
    assert(p.turbineRatedShaftPowerEach_kW>0);
    assert(p.generatorCount>=0 && mod(p.generatorCount,1)==0 && p.generatorMassEach_kg>=0);
    assert(p.fuelTankCount>=1 && mod(p.fuelTankCount,1)==0 && p.fuelDensity_kgm3>0);
    assert(p.integralTankVolumeFraction>=0 && p.integralTankVolumeFraction<=1);
    assert(p.wingFuelFraction>=0 && p.wingFuelFraction<=1 && p.trappedFuel_kg>=0);
    assert(p.growthFraction>=0 && p.unidentifiedFraction>=0 && p.fuelContingencyFraction>=0);
    b=p.driveBanks; assert(size(b,2)==6 && ~isempty(b) && all(isfinite(b(:))));
    assert(all(b(:,1)>=0 & mod(b(:,1),1)==0) && all(all(b(:,2:4)>0)) && all(all(b(:,5:6)>=0)));
    assert(numel(p.extraMass_kg)==numel(p.extraNames) && numel(p.extraMass_kg)==numel(p.extraX_m));
    assert(numel(p.extraMass_kg)==numel(p.extraGroups) && all(p.extraMass_kg>=0));
    assert(all(ismember(p.extraGroups,{'Structure','Propulsion','Equipment'})));
    assert(all(structfun(@(v) isscalar(v)&&isfinite(v)&&v>0,p.factor)));
    assert(all(structfun(@(v) isscalar(v)&&isfinite(v),p.x)) && all(isfinite(p.extraX_m)));
    assert(p.hydraulicsKh>=0 && p.hydraulicsMach>=0 && p.uninstalledAvionics_kg>=0);
end