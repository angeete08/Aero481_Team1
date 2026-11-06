%% SINGULAR INPUT FILE & RUNNER

% Conversion and base weights
in.kgToLb = 1 / 0.45359237;
in.mTO    = 3757;    % Initial estimate [kg]

%% ========================================================================
% 1. MISSION PARAMETERS & PAYLOAD
% ========================================================================
in.range_nmi         = 400;     % nmi
in.cruise_kt         = 225;     % kts
in.altitude_ft       = 12000;   % ft
in.serviceCeiling    = 12500;   % ft
in.LD                = 16.5;    % Initial estimate
in.num_people        = 8;
in.is_electric       = 1;
in.electricRange_nmi = 200;     % nmi

in.personMass_kg = 86.1825;          
in.baggagePerPassenger_kg = 13.6075;
in.crewBaggage_kg = 13.6075;
in.crew = 1;

in.climb_kt          = 120;     % kts
in.climb_ftmin       = 1500;    % ft/min
in.descent_kt        = 150;     % kts
in.descent_ftmin     = 1300;    % ft/min

in.alternate_nmi     = 50;      % Alternate distance [nmi]
in.reserve_min       = 30;      % Reserve duration [min]
in.reserve_kt        = 0.80 * in.cruise_kt;

in.engineOut_nmi     = 20;      % Additional battery-only range [nmi]
in.engineOut_kt      = 120;     % Contingency speed [kt]

in.climbDistance_nmi = in.climb_kt * (in.altitude_ft / in.climb_ftmin) / 60;
in.descentDistance_nmi = in.descent_kt * (in.altitude_ft / in.descent_ftmin) / 60;

in.Cp                = 0.40;    % [lb/(hp*hr)]
in.etaProp           = 0.85;    % Propeller efficiency
in.K_Breguet         = 325.9;

% Payload
in.mPeople_kg        = 86.1825 * in.num_people;
in.mBaggage_kg       = 13.6075 * in.num_people;

%% ========================================================================
% 2. RFP REQUIREMENTS - SI UNITS
% ========================================================================
in.g0                   = 9.80665;
in.Vcruise_threshold_ms = 225 * 0.514444;     % [m/s]
in.Vcruise_objective_ms = 250 * 0.514444;     % [m/s]
in.s_TO_req_m           = 300 * 0.3048;       % 91.44 m
in.s_L_req_m            = 300 * 0.3048;       % 91.44 m
in.ROC_RFP_ms           = in.climb_ftmin * 0.00508;     % 7.62 m/s
in.h_obstacle_m         = 50 * 0.3048;        % 15.24 m

% Required field condition: sea level, ISA + 18 F = ISA + 10 C
in.h_field_m            = 0;
in.dT_field_C           = 10;

% RFP 5000-ft hot-day reporting condition
in.h_5000_m             = 5000 * 0.3048;      % 1524 m
in.dT_5000_C            = 10;

in.h_cruise_m           = in.altitude_ft * 0.3048;      
in.dT_cruise_C          = 0;
in.h_ceiling_m          = in.serviceCeiling * 0.3048;
in.dT_ceiling_C         = 0;
in.ROC_ceiling_ms       = 100 * 0.00508;

in.MTOW_limit_kg        = 19000 * 0.45359237;
in.isHighSpeed = false;

%% ========================================================================
% 3. SELECTED DESIGN ASSUMPTIONS & AERODYNAMICS
% ========================================================================
in.AR                   = 8.0;                % aspect ratio 

% Weight fractions
in.beta_cruise          = 0.97;               
in.beta_landing         = 0.97;               
in.beta_climb           = 1.00;
in.beta_maneuver        = 0.97;
in.beta_ceiling         = 0.97;
in.Cp_lb_hphr           = 0.40;
in.electricInstallationFraction = 0.30;

% High-lift selections
in.CLmax_clean          = 1.10;
in.CLmax_TO             = 7.5;
in.CLmax_L              = 7.5;

% Parasite-drag method
in.wet_c_SI             = -0.84050366;
in.wet_d                =  0.8099;
in.Cf_equiv             = 0.0030;
in.CD0_ref              = 0.018;
in.LD_ref               = 8.2;

% Drag increments
in.dCD0_TO_flaps        = 0.015;    
in.dCD0_L_flaps         = 0.065;
in.dCD0_gear            = 0.020;
in.dCD0_CLoT            = 0.010;

% Span efficiencies
in.e_clean              = 0.825;       
in.e_TO                 = 0.775;
in.e_L                  = 0.725;
in.e_CLoT               = 0.75;

% Propeller efficiencies / available power
in.eta_TO               = 0.75;
in.eta_climb            = 0.80;
in.eta_cruise           = 0.85;
in.eta_balked           = 0.75;
in.eta_maneuver         = 0.85;
in.eta_ceiling          = 0.85;

in.powerFrac_TO         = 1.00;
in.powerFrac_climb      = 1.00;
in.powerFrac_cruise     = 0.94; 
in.powerFrac_balked     = 1.00;
in.powerFrac_maneuver   = 0.80;               
in.powerFrac_ceiling    = 0.70;               

% Critical Loss of Thrust (CLoT) & Climbs
in.CLoT_power_remaining = 0.50;
in.G_initial            = 0.04;
in.G_CLoT               = 0.01;
in.G_balked             = 0.03;

% Stall speed safety margins
in.ks_takeoff           = 1.10;
in.ks_initial           = 1.20;
in.ks_CLoT              = 1.20;
in.ks_balked            = 1.20;
in.ks_landing           = 1.20;
in.ks_ceiling           = 1.10;

% STOL Takeoff & Landing Assumptions
in.mu_roll              = 0.02;               
in.k_rotation           = 1.10;               
in.k_transition         = 1.15;               
in.gamma_takeoff_deg    = 15;                 
in.n_transition         = 1.50;               

in.gamma_approach_deg   = 15;
in.k_touchdown          = 1.15;               
in.stop_decel_g         = 0.69;               
in.n_flare              = 1.15;              

% Structural Sizing Parameters (Raymer)
in.Nz_ultimate          = 5.7;
in.tc_root              = 0.16;
in.taperRatio           = 0.7;
in.sweep25_deg          = 0;
in.controlSurfaceFraction = 0.30;
in.kPropulsionGroup     = 1.16;

% Component efficiencies & hardware architecture
in.etaMotor             = 0.96;
in.etaWiring            = 0.98;
in.etaInverter          = 0.98;
in.etaCable             = 0.996;
in.etaBattery           = 0.96;
in.etaPropElec          = 0.85;
in.etaController        = 0.98;

in.thermalShare         = 0.50;   
in.electricShare        = 0.50;   
in.engineOfftakeFraction = 0.05;
in.hubMotorFractionOfElectric = 0.20;  
in.distributedFractionOfElectric = 0.80; 
in.numDistributedMotors = 8;
in.generatorFractionOfElectric = 0.30;
in.generator_kWkg       = 8.0;
in.generatorInstallationFraction = 0.15;

in.electricShaft_kW     = 550;  
in.boost_min            = 5;    
in.landing_min          = 0;    
in.landingPowerFraction = 0.25; 
in.auxiliary_kW         = 3;    

in.pack_Whkg            = 700;
in.usableSOC            = 0.85;    
in.pack_kWkg            = 2.0;     

in.motor_kWkg           = 10;      
in.inverter_kWkg        = 16;      
in.installationFraction = 0.30;
in.extraElectricHardware_kg = 5;

% Mission-segment fractions
in.rWarmup  = 0.990;
in.rTaxi    = 0.995;
in.rTakeoff = 0.995;
in.rClimb   = 0.985;
in.rDescent = 0.985;
in.rLanding = 0.995;

in.weightTolerance      = 1e-7;
in.maxWeightIterations  = 1000;
in.massNumericalLimit_kg = 25000;

% Maneuver requirements
in.phi_maneuver_deg     = 30;
in.n_maneuver           = 1/cosd(in.phi_maneuver_deg);
in.V_maneuver_ms = in.Vcruise_threshold_ms;

%% ========================================================================
% 4. ENVIRONMENTAL OBJECTIVE - SIMPLE CLIMATE PROXY
% ========================================================================
in.fuelCO2_kg_per_kgFuel                    = 3.16;
in.gridCO2_kg_per_kWh                       = 0.348018746;
in.batteryManufacturing_kgCO2e_per_kWh      = 64;
in.airframeManufacturing_kgCO2e_per_kgEmpty = 1401164 / 42092;
in.gridChargeEfficiency                     = 0.95;
in.lifeMissions                             = 15000;

% Tail Sizing 
in.ht.taper = 1;
in.ht.tc = 0.12; 
in.ht.sweep_deg = 0;

in.vt.taper = 0.325;
in.vt.tc = 0.12; 
in.vt.sweep_deg = 30;

% Fuselage Size
in.tailHeightRatio = 0;
in.fuselageWettedArea_m2 = 130;
in.fuselageStructuralLength_m = 13.68;
in.fuselageDepth_m = 2.4;
in.tailArm_m = 8.6;            
in.pressurizedVolume_m3 = 0;
in.cabinDifferentialPressure_Pa = 0;
in.mainGearLength_m = 1.1;     
in.noseGearLength_m = 0.9;
in.retractableGear = true;

% Loads and aerodynamic state
in.limitFlightLoadFactor = 3.8; 
in.ultimateLandingLoadFactor = 4.5;
in.flightDesignMassFraction = 1.0; 
in.landingDesignMassFraction = 1.0; 
in.cruiseSpeed_mps = in.cruise_kt * 0.514444;
in.cruiseDensity_kgm3 = 0.849;
in.maximumMach = 0.40;

% Factors
in.factor.wing = 0.85; 
in.factor.ht = 1; 
in.factor.vt = 1;
in.factor.fuselage = 1; 
in.factor.gear = 1;
in.factor.flightControls = 1;
in.growthFraction = 0.05;      
in.unidentifiedFraction = 0;  

% Fuel Assumptions
in.fuelDensity_kgm3 = 800;
in.fuelTankCount = 2;
in.integralTankVolumeFraction = 1; 
in.wingFuelFraction = 1;       
in.trappedFuel_kg = 5;  

in.fuelContingencyFraction = 0.05; 
in.energyReferenceMass_kg = 5000;
in.missionPowerMassExponent = 0;

in.battery.minimumInstalledCapacity_kWh = 0;

in.turbineCount = 1;
in.turbineMassEach_kg = 180;   
in.generatorCount = 0;        
in.generatorMassEach_kg = 0;

%% Systems & Equipment Inputs
in.uninstalledAvionics_kg = 35;
in.hydraulicsKh           = 0;           
in.hydraulicsMach         = 0.1;       
in.useRaymerElectrical    = true;
in.conventionalElectrical_kg = 35; 
in.useRaymerEnvironmental = false;
in.environmentalAndAntiIce_kg = 45; 
in.useRaymerFurnishings   = false;
in.furnishings_kg         = 140;      

%% Longitudinal Stations (Datum Positions in meters)
in.x.wing        = 3.5; 
in.x.ht          = 8.5; 
in.x.vt          = 8.5; 
in.x.fuselage    = 4.2;
in.x.mainGear    = 4.0; 
in.x.noseGear    = 1.0; 
in.x.turbine     = 2.0; 
in.x.generator   = 2.5;
in.x.drives      = 3.2; 
in.x.battery     = 4.0; 
in.x.fuel        = 3.5; 
in.x.controls    = 4.0;
in.x.systems     = 3.0; 
in.x.furnishings = 4.5; 
in.x.crew        = 1.8;
in.x.passengers  = 4.5; 
in.x.baggage     = 6.5; 
in.x.cargo       = 6.5;
in.x.otherUseful = 4.0;

in.wingLEMAC_m   = 3.0;


