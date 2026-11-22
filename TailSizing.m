function TailData = TailSizing(in, S_design_m2)
%% ========================================================================
%  MODULAR EMPENNAGE (TAIL) SIZING FUNCTION
%  Computes tail areas, spans, and chords based on pipeline wing geometry
%  (tapered wing, untapered rectangular tail surfaces)
%  ========================================================================

% Wing geometry derived from pipeline
AR = in.AR;
b_wing = sqrt(AR * S_design_m2); % Wing Span [m]

% Wing taper ratio (defaults to 1.0 = rectangular if omitted)
if isfield(in, 'taperRatio'), lambda_w = in.taperRatio; else, lambda_w = 1.0; end
c_wing_root = (2 * S_design_m2) / (b_wing * (1 + lambda_w));
c_wing_tip  = lambda_w * c_wing_root;
c_wing_mac  = (2/3) * c_wing_root * (1 + lambda_w + lambda_w^2) / (1 + lambda_w);

% Tail moment arms (default or unpacked from 'in')
if isfield(in, 'Lh'), Lh = in.Lh; else, Lh = 8.0; end % [m]
if isfield(in, 'Lv'), Lv = in.Lv; else, Lv = 8.0; end % [m]

% Tail volume coefficients
if isfield(in, 'Vh'), Vh = in.Vh; else, Vh = 0.90; end
if isfield(in, 'Vv'), Vv = in.Vv; else, Vv = 0.06; end

% Control surface chord ratios
elevator_chord_ratio = 0.30;
rudder_chord_ratio   = 0.40;

% Target Aspect Ratios for Tail Surfaces
AR_h_target = 3.5;
AR_v_target = 1.8;

% Calculate Horizontal Stabilizer Area (using wing MAC)
Sh = (Vh * S_design_m2 * c_wing_mac) ./ Lh;

% Calculate Vertical Stabilizer Area
Sv = (Vv * S_design_m2 * b_wing) ./ Lv;

% Horizontal Stabilizer Dimensions (untapered / rectangular)
b_h = sqrt(AR_h_target * Sh);
c_h = Sh / b_h;
elevator_chord = c_h * elevator_chord_ratio;

% Vertical Stabilizer Dimensions (untapered / rectangular)
h_v = sqrt(AR_v_target * Sv);
c_v = Sv / h_v;
rudder_chord = c_v * rudder_chord_ratio;

TailData = [Sh, b_h, c_h, Sv, h_v, c_v];

% Print Summary Report
fprintf('\n--- EMPENNAGE DESIGN RESULTS (PIPELINE INTEGRATED) ---\n');
fprintf('Based on Wing Span: %.2f m, MAC: %.2f m (lambda_w = %.2f)\n', ...
    b_wing, c_wing_mac, lambda_w);
fprintf(' - Wing Root / Tip Chord: %.2f m / %.2f m\n', c_wing_root, c_wing_tip);
fprintf('Wing Area (Sw): %.4f m^2\n', S_design_m2);
fprintf('Tail Moment Arms (Lh, Lv): %.3f m, %.3f m\n\n', Lh, Lv);

fprintf('--- HORIZONTAL STABILIZER (UNTAPERED) ---\n');
fprintf('Required Area (Sh): %.4f m^2\n', Sh);
fprintf('Suggested Dimensions -> Span: %.1f m, Chord: %.1f m\n', b_h, c_h);
fprintf(' - Elevator Chord: %.1f m\n\n', elevator_chord);

fprintf('--- VERTICAL STABILIZER (UNTAPERED) ---\n');
fprintf('Required Area (Sv): %.4f m^2\n', Sv);
fprintf('Suggested Dimensions -> Height: %.1f m, Chord: %.1f m\n', h_v, c_v);
fprintf(' - Rudder Chord: %.1f m\n\n', rudder_chord);
end