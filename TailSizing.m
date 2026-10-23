clear; clc; close all;

b_wing = 22.02362; % Wing Span [m]
c_wing = 3.6706; % Wing Chord (since it's a constant chord/rectangular wing) [m]

Lh = 8; % Horizontal tail moment arm [m]
Lv = 8; % Vertical tail moment arm [m] 

% Used twin turboprop for this
Vh = 0.90; % Horizontal Tail Volume Coefficient 
Vv = 0.06; % Vertical Tail Volume Coefficient  %Decreased by 0.01 from raymers

% Control Surface Ratios
elevator_chord_ratio = 0.30; % Elevator chord as a % of horizontal stab chord (25-30%)
rudder_chord_ratio = 0.40; % Rudder chord as a % of vertical stab chord (30-40%)

% Target Aspect Ratios for Tail Surfaces (for suggesting dimensions)
AR_h_target = 3.5; % Typical range 3-4
AR_v_target = 1.8; % Typical range 1.3-2 (for vertical stabs, height^2/area)

% Calculate Wing Area
Sw = b_wing * c_wing;

% ---- Horizontal Stabilizer Sizing ----
% Vh = (Sh * Lh) / (Sw * c_wing) -> solve for Sh
Sh = (Vh * Sw * c_wing) ./ Lh;

% ---- Vertical Stabilizer Sizing ----
% Vv = (Sv * Lv) / (Sw * b_wing) -> solve for Sv
Sv = (Vv * Sw * b_wing) ./ Lv;


%% 4. SUGGEST PRACTICAL DIMENSIONS AND CONTROL SURFACES
% Using the target aspect ratio to suggest a practical span and chord.
% AR = span^2 / area => span = sqrt(AR * area)

% Horizontal Stabilizer
b_h = sqrt(AR_h_target * Sh); % Suggested span of horizontal stabilizer
c_h = Sh / b_h; % Suggested chord of horizontal stabilizer
elevator_chord = c_h * elevator_chord_ratio;

% Vertical Stabilizer (height is its "span")
h_v = sqrt(AR_v_target * Sv); % Suggested height of vertical stabilizer
c_v = Sv / h_v; % Suggested chord of vertical stabilizer
rudder_chord = c_v * rudder_chord_ratio;

fprintf('--- EMPENNAGE DESIGN RESULTS ---\n\n');
fprintf('Based on Wing Span: %.2f m and Chord: %.2f m\n', b_wing, c_wing);
fprintf('Wing Area (Sw): %.4f m^2\n', Sw);
fprintf('Tail Moment Arm (Lh, Lv): %.3f m\n\n', Lh);

fprintf('--- HORIZONTAL STABILIZER ---\n');
fprintf('Required Area (Sh): %.4f m^2\n', Sh);
fprintf('Suggested Dimensions -> Span: %.1f m, Chord: %.1f m\n', b_h, c_h);
fprintf(' - Elevator Chord: %.1f m\n\n', elevator_chord);

fprintf('--- VERTICAL STABILIZER ---\n');
fprintf('Required Area (Sv): %.4f m^2\n', Sv);
fprintf('Suggested Dimensions -> Height: %.1f m, Chord: %.1f m\n', h_v, c_v);
fprintf(' - Rudder Chord: %.1f m\n\n', rudder_chord);