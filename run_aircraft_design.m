%% ========================================================================
%  MASTER AIRCRAFT DESIGN & CONSTRAINT WORKFLOW RUNNER
%  ========================================================================
clear;
clc;
close all;

fprintf('1. Initializing aircraft inputs...\n');
aircraft_inputs; 

fprintf('2. Running fixed-point weight prediction...\n');
values_kg = Weight_Predict(in);

% Calculate weights in pounds for display
kgToLb = in.kgToLb;
values_lb = values_kg * kgToLb;

% Display clean named weight summary table
ResultsTable = table( ...
    values_kg', ...
    values_lb', ...
    'VariableNames', {'Mass_kg', 'Mass_lb'}, ...
    'RowNames', { ...
    'GrossTakeoff', ...
    'Empty', ...
    'Fuel', ...
    'CrewPassengers', ...
    'Baggage', ...
    'ElectricHardware', ...
    'Battery'} );
disp('========================================');
disp('      AIRCRAFT WEIGHT SIZING RESULTS    ');
disp('========================================');
disp(ResultsTable);

fprintf('3. Generating P/W - W/S constraint diagram and performance outputs...\n');
% Run constraint diagram (ensure it does NOT start with 'clear')
AE481_Constraint_Diagram_v9;

%% ========================================================================
% 4. PARAMETRIC P-S AND ENVIRONMENTAL STUDY (Weight_Predict_PS)
% ========================================================================
fprintf('\n4. Initializing parameter structure `p` from centralized inputs...\n');

% Map input structure fields directly to parameter structure `p`
p = in; 
p.mTO_baseline_kg      = values_kg(1);
p.mEmpty_baseline_kg   = values_kg(2);
p.mElectric_baseline_kg= values_kg(6);
p.mBattery_baseline_kg = values_kg(7);
p.S_ref_m2             = S_design_m2;
p.P_ref_kW             = P_installed_kW;
p.Swet_rest_m2         = ((p.CD0_ref / p.Cf_equiv) - 2) * p.S_ref_m2;

% Evaluate parametric weight model at the selected design point
result_PS = Weight_Predict_PS(S_design_m2, P_installed_kW, p);
disp('Parametric Weight Model Results (Weight_Predict_PS):');
disp(result_PS);

fprintf('\n5. Running AE481_PS_Environmental_Parametric_v8 workflow...\n');
AE481_PS_Environmental_Parametric_v8;

fprintf('\n6. Running TailSizing workflow...\n');
if exist('optS', 'var') && ~isempty(optS)
    finalWingArea_m2 = optS;
else
    finalWingArea_m2 = S_design_m2;
end

if exist('optP', 'var') && ~isempty(optP)
    finalPower_kW = optP;
else
    finalPower_kW = P_installed_kW;
end

TailData = TailSizing(in, finalWingArea_m2);

fprintf('\n7. Running Refined Raymer Weight Prediction (Hybrid_Aircraft_Refined_Weight)...\n');
refined_values_kg = Hybrid_Aircraft_Refined_Weight(in, TailData, finalWingArea_m2, finalPower_kW);
refined_values_lb = refined_values_kg * kgToLb;

RefinedResultsTable = table( ...
    refined_values_kg', ...
    refined_values_lb', ...
    'VariableNames', {'Mass_kg', 'Mass_lb'}, ...
    'RowNames', { ...
    'GrossTakeoff', ...
    'Empty', ...
    'Fuel', ...
    'CrewPassengers', ...
    'Baggage', ...
    'ElectricHardware', ...
    'Battery'} );

disp('========================================');
disp('   REFINED RAYMER WEIGHT SIZING RESULTS ');
disp('========================================');
disp(RefinedResultsTable);

values_kg = refined_values_kg;

fprintf('\n8. Rerunning Constraint Diagram for P/W vs W/N \n');
% Rerun constraint diagram with the updated refined weights
AE481_Constraint_Diagram_v9;

fprintf('\n9. Updating parameter structure `p` with refined weights, wing area, and power...\n');

% Map the updated workflow parameters into `p`
p = in;
p.mTO_baseline_kg       = refined_values_kg(1);
p.mEmpty_baseline_kg    = refined_values_kg(2);
p.mElectric_baseline_kg = refined_values_kg(6);
p.mBattery_baseline_kg  = refined_values_kg(7);

% Use your final optimized wing area and power from the previous steps
if exist('optS', 'var') && ~isempty(optS)
    p.S_ref_m2 = optS;
else
    p.S_ref_m2 = finalWingArea_m2;
end

if exist('optP', 'var') && ~isempty(optP)
    p.P_ref_kW = optP;
else
    p.P_ref_kW = finalPower_kW;
end

% Recompute structural wet area anchor reference if needed
Wbase_N = p.mTO_baseline_kg * p.g0;
p.Swet_rest_m2 = (p.CD0_ref / p.Cf_equiv) * p.S_ref_m2 - 2 * p.S_ref_m2;

% Evaluate parametric weight model at the selected design point
result_PS = Weight_Predict_PS(S_design_m2, P_installed_kW, p);
disp('Parametric Weight Model Results (Weight_Predict_PS):');
disp(result_PS)

fprintf('\n10. Rerunning AE481_PS_Environmental_Parametric_v8 with updated baseline...\n');
AE481_PS_Environmental_Parametric_v8;

fprintf('\nFull refined workflow complete! All updated P-S sweeps, candidates, and figures generated successfully.\n');

fprintf('\n11. Calling TailSizing workflow again with the final updated wing area...\n');

if exist('optS', 'var') && ~isempty(optS)
    finalWingArea_m2 = optS;
end

% Re-evaluate tail sizing and geometry dimensions using the final optimized S
TailData = TailSizing(in, finalWingArea_m2);

fprintf('\nFull refined workflow complete! All updated P-S sweeps, tail sizing dimensions, and figures generated successfully.\n');

fprintf('\nWorkflow complete! All plots and reports generated successfully.\n');

