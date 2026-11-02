% AE481 - Preliminary Hybrid-Electric Weight Estimation Comparison to known
% aircraft to check validity
%
% Calculates the seven columns required for "Weights - Overall".
%
% METHOD:
%   1. Empty-weight regression          -> AE481 slides 23-25
%   2. Mission fuel fractions           -> AE481 slides 27-31
%   3. Propeller Breguet range          -> AE481 slides 41-47
%   4. Fixed-point takeoff-weight loop  -> AE481 slides 49-51
%   5. Electric propulsion sizing       -> AE481 slides 63-70
%
% NOTE:
% Values such as battery specific energy, L/D, mission range, and
% installed electric power are DESIGN ASSUMPTIONS. The calculation
% method itself comes from the lecture.
%
clear;
close all;
clc;

%% Validate Weight Estimation Code
%{
Use a comparison aircraft to validate your weight estimation and cost estimation codes. This
means using the specifications of other aircraft with your model and code to check if the
results are reasonable and close to the actual values. Comment on the differences between
your weight estimates and the listed values.

%}

% --- Our Aircraft ---
%MBraer = Weight_Predict(400, 225, 20000, 16, 8, 'electric', 210);

%Weight_Predict(range_nmi, cruise_kt, altitude_ft, LD, num_people, text, electricRange_nmi)

% --- Comparison Aircraft ---
%Electra_EL9 = Weight_Predict(330, 175, 12500, 12, 10, 'electric', 0);
%PA_18  = Weight_Predict(400, 100, 19000, 10, 2, 'prop super cub', 0);
Kodiak_100 = Weight_Predict(1132, 175, 25000, 12, 9, 'prop', 0);
%PC_6  = Weight_Predict(488, 145, 25000, 10, 11, 'prop', 0);
%P2012 = Weight_Predict(950, 175, 19500, 12.5, 11, 'twin prop', 0);