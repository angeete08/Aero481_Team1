area_per_chord = 0.0145;
b = 3.41;
S = 92.94;
wing_volume = 92.94 * 3.41 * 0.0145; %m^3
disp(wing_volume);
usable_vol_fraction = 0.4;
required_fuel_vol = 1.325;
extra_fuel_vol = (0.4*wing_volume) - 1.325;
extra_fuel_amount = 804 * extra_fuel_vol;
disp(extra_fuel_amount);

