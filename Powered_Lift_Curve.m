%% POWERED LIFT CURVE - AE481 preliminary blown-wing model
% Run this complete script (MATLAB R2016b or newer; no toolboxes required).
% All design assumptions are in Section 1. Defaults are illustrative,
% NOT the final dimensions, power, or weight of your aircraft.
%
% Source: 10b-Blown_Wing_Hybrid_eSTOL.pdf, PDF pages 12-15, 30, 34, 37.
% Method: actuator disk -> momentum excess -> jet turning -> lift increment
%         -> separate powered-stall angle and operating margin.
% Fit: AR = 7, approximately uniform blowing, alpha -5 to 20 deg,
%      effective jet angle 0 to 70 deg, momentum coefficient 0 to 25.
% The lift fit is linear in alpha: it DOES NOT predict post-stall behavior.
% This script predicts lift curves, not takeoff/landing distance or trim.

clear; clc; close all;

%% 1. EDITABLE ASSUMPTIONS
% Flight condition (held fixed while plotting lift versus angle of attack)
% V_kt = 25.95;                    % Aircraft airspeed, knots (must be > 0)
V_kt = 27.6;                    % Aircraft airspeed, knots (must be > 0)

rho = 1.1839;                 % Air density, kg/m^3

% Wing and propeller geometry
S = 91.74;                       % Wing reference area, m^2
AR = 6;                       % Aspect ratio (fit is calibrated at AR = 7)
Nprop = 8;                    % Number of identical blowing propellers
Dprop = 1.98;          % Diameter per propeller, m; defaults give Ad/S ~ 0.5
flap_deg = 60;                % Geometric flap deflection, degrees

% Total shaft power delivered to ALL blowing propellers, in kW.
% Use a scalar for one curve, or a vector to compare power settings.
% Zero gives the unpowered baseline. Power is AFTER drivetrain losses.
Pshaft_kW = [1200];
eta_a = 0.84;                 % Ideal disk power / actual shaft power

% Unpowered lift curve: teaching baseline from the slides.
% Replace this function with your own UNPOWERED, flapped wing lift curve
% when available. It must accept a vector of alpha values in DEGREES.
CL_unpowered = @(alpha, flap) 0.1856 + 0.0334*flap + 0.0667*alpha;

% Powered-stall approximation and operating margin from the slides.
alpha_stall0_deg = 16;
stall_gain_deg = 1.5;         % Increase in stall angle per unit T_c'
alpha_stall_cap_deg = 22;
margin_deg = 8;               % 8: takeoff example; 10: approach example
% Any rotation, trim, or control limit can impose a lower usable alpha.
alpha_control_limit_deg = Inf; % Replace Inf if a justified limit is known

% Optional steady-flight lift requirement. Leave mass_kg = NaN to disable.
% This checks NORMAL FORCE ONLY; a descent also requires a net-force check.
mass_kg = NaN;               % Aircraft mass in this flight condition, kg
gamma_deg = 0;               % Steady flight-path angle, degrees

%% 2. UNITS, GEOMETRY, AND INPUT CHECKS
V = V_kt * 0.514444;          % m/s
Pshaft_W = Pshaft_kW(:) * 1000;
Ad = Nprop * pi * Dprop^2 / 4;
sigma = Ad / S;
q = 0.5 * rho * V^2;

assert(isfinite(V) && V > 0, 'Use a positive airspeed for a CL curve.');
assert(rho > 0 && S > 0 && AR > 0 && Dprop > 0, ...
    'Density and geometry must be positive.');
assert(Nprop >= 1 && Nprop == round(Nprop), 'Nprop must be a positive integer.');
assert(~isempty(Pshaft_W) && all(isfinite(Pshaft_W) & Pshaft_W >= 0), ...
    'Shaft powers must be finite and nonnegative.');
assert(eta_a > 0 && eta_a <= 1, 'eta_a must be in (0,1].');
assert(sigma > 0 && sigma <= 1, ...
    'The Appendix A5 turning table covers 0 <= Ad/S <= 1.');
assert(flap_deg >= 0 && flap_deg <= 70, 'Use flap deflections from 0 to 70 deg.');
assert(margin_deg >= 0 && isfinite(margin_deg), 'Margin must be nonnegative.');
assert(isnan(mass_kg) || (isfinite(mass_kg) && mass_kg > 0), ...
    'mass_kg must be positive or NaN.');
assert(isfinite(gamma_deg) && abs(gamma_deg) < 90, 'Use |gamma| < 90 deg.');
if abs(AR - 7) > 0.01
    warning(['The powered increment was fitted at AR = 7. Changing AR here ' ...
        'does not recalibrate the lift fit for your wing.']);
end

%% 3. EFFECTIVE JET TURNING - APPENDIX A5
% Interpolate in disk-area ratio, then blend between 40- and 60-deg flaps.
% Do not use the problematic thesis turning fit mentioned in Appendix A9.
eta_F = turningEfficiency(sigma, flap_deg);
deltaJ_deg = eta_F * flap_deg;
assert(deltaJ_deg <= 70, 'Effective jet angle exceeds the lift-fit range.');

%% 4. CALCULATE BLOWING, LIFT CURVES, AND ANGLE LIMITS
nCase = numel(Pshaft_W);
alpha_deg = linspace(-5, 20, 251)';
Vj = zeros(nCase,1); T = Vj; Tc = Vj; deltaCJ = Vj;
alpha_stall = Vj; alpha_allow = Vj; CL_allow = Vj;
CL_max_model = nan(nCase,1); % Remains NaN if stall lies outside fit domain
CL_at_curve_end = Vj;
CL_curve = nan(numel(alpha_deg), nCase);

for k = 1:nCase
    % Solve the actuator-disk power balance for the physical root Vj >= V.
    if Pshaft_W(k) == 0
        Vj(k) = V;
    else
        powerBalance = @(vj) rho*Ad/4 * (vj-V)*(vj+V)^2 ...
            - eta_a*Pshaft_W(k);
        dV_bound = (4*eta_a*Pshaft_W(k)/(rho*Ad))^(1/3);
        Vj(k) = fzero(powerBalance, [V, V+dV_bound]);
    end

    T(k) = rho*Ad/2 * (Vj(k)^2 - V^2);
    Tc(k) = T(k)/(q*S);
    deltaCJ(k) = Tc(k)*(1 + V/Vj(k));
    if deltaCJ(k) > 25
        error(['Power case %d gives deltaCJ = %.2f > 25. ' ...
            'Outside lift-fit range; do not silently clip it.'], k, deltaCJ(k));
    end

    % Baseline uses GEOMETRIC flap angle; increment uses EFFECTIVE jet angle.
    liftAt = @(a) CL_unpowered(a, flap_deg) ...
        + poweredIncrement(a, deltaJ_deg, deltaCJ(k));

    alpha_stall(k) = min(alpha_stall0_deg + stall_gain_deg*Tc(k), ...
        alpha_stall_cap_deg);
    alpha_allow(k) = min(alpha_stall(k)-margin_deg, alpha_control_limit_deg);

    % Do not evaluate lift outside its alpha domain or beyond modeled stall.
    alpha_end = min(alpha_stall(k), 20);
    assert(alpha_allow(k) >= -5 && alpha_allow(k) <= alpha_end, ...
        'Allowable alpha is outside the supported lift-curve domain.');
    use = alpha_deg <= alpha_end;
    CL_curve(use,k) = liftAt(alpha_deg(use));
    CL_allow(k) = liftAt(alpha_allow(k));
    CL_at_curve_end(k) = liftAt(alpha_end);
    if alpha_stall(k) >= -5 && alpha_stall(k) <= 20
        CL_max_model(k) = liftAt(alpha_stall(k));
    end
end

%% 5. OPTIONAL REQUIRED LIFT AND NUMERICAL RESULTS
CL_required = NaN;
if ~isnan(mass_kg)
    W = mass_kg * 9.80665;     % Convert mass to WEIGHT, newtons
    CL_required = W*cosd(gamma_deg)/(q*S);
end

results = table(Pshaft_W/1000, Vj, T, Tc, deltaCJ, alpha_stall, ...
    alpha_allow, CL_allow, CL_max_model, CL_at_curve_end, ...
    'VariableNames', {'Pshaft_kW','Vj_mps','Thrust_N','Tc_prime', ...
    'deltaCJ','alphaStall_deg','alphaAllow_deg','CL_allow', ...
    'CLmax_model','CL_at_curve_end'});

fprintf('\nPRELIMINARY POWERED LIFT MODEL\n');
fprintf('V = %.2f kt; rho = %.4f kg/m^3; q = %.2f Pa\n', V_kt, rho, q);
fprintf('S = %.2f m^2; Ad = %.2f m^2; Ad/S = %.4f\n', S, Ad, sigma);
fprintf('Flap = %.1f deg; eta_F = %.4f; jet turn = %.2f deg\n', ...
    flap_deg, eta_F, deltaJ_deg);
fprintf('Angle-of-attack margin = %.1f deg\n\n', margin_deg);
disp(results);
fprintf(['CLmax_model = NaN means the assumed stall is outside the lift-fit\n' ...
    'alpha range. CL_at_curve_end is then NOT a predicted maximum.\n']);
if ~isnan(CL_required)
    results.CL_required = repmat(CL_required,nCase,1);
    results.CL_margin = CL_allow - CL_required;
    fprintf('\nRequired steady-flight CL = %.3f\n', CL_required);
    fprintf('Positive CL_margin means enough normal force at allowable alpha.\n');
    disp(results(:,{'Pshaft_kW','CL_allow','CL_required','CL_margin'}));
end

%% 6. PLOT LIFT VERSUS ANGLE OF ATTACK
% Solid line: within selected operating limit.
% Dashed line: above operating limit, up to stall or the fit boundary.
figure('Color','w','Name','Powered lift curve');
hold on; grid on; box on;
colors = lines(nCase);
for k = 1:nCase
    liftAt = @(a) CL_unpowered(a, flap_deg) ...
        + poweredIncrement(a, deltaJ_deg, deltaCJ(k));
    a1 = linspace(-5, alpha_allow(k), 150);
    a2 = linspace(alpha_allow(k), min(alpha_stall(k),20), 80);
    plot(a1, liftAt(a1), '-', 'Color', colors(k,:), 'LineWidth', 2, ...
        'DisplayName', sprintf('Shaft power: %.0f kW',Pshaft_W(k)/1000));
    plot(a2, liftAt(a2), '--', 'Color', colors(k,:), 'LineWidth', 1.4, ...
        'HandleVisibility','off');
    plot(alpha_allow(k), CL_allow(k), 'o', 'Color', colors(k,:), ...
        'MarkerFaceColor', colors(k,:), 'MarkerSize', 7, 'HandleVisibility','off');
    if ~isnan(CL_max_model(k))
        plot(alpha_stall(k), CL_max_model(k), 'x', 'Color', colors(k,:), ...
            'LineWidth', 2, 'MarkerSize', 9, 'HandleVisibility','off');
    else
        plot(20, CL_at_curve_end(k), 's', 'Color', colors(k,:), ...
            'MarkerSize', 7, 'LineWidth', 1.5, 'HandleVisibility','off');
    end
end
% Dummy handles explain markers without adding repeated legend entries.
plot(NaN,NaN,'ko','MarkerFaceColor','k','DisplayName','Operating limit');
plot(NaN,NaN,'kx','LineWidth',2,'DisplayName','Modeled stall');
if any(isnan(CL_max_model))
    plot(NaN,NaN,'ks','DisplayName','Fit boundary; maximum not established');
end
if ~isnan(CL_required)
    plot([-5 20], [CL_required CL_required], 'k:', 'LineWidth',1.5, ...
        'DisplayName','Required steady-flight C_L');
end
xlabel('Angle of attack, \alpha (deg)');
ylabel('Effective aircraft lift coefficient, C_L');
% title({sprintf('Powered lift at %.1f kt, flap %.0f deg, A_d/S = %.3f', ...
%     V_kt, flap_deg, sigma), ...
%     sprintf('Solid: operating range | Dashed: margin to stall/fit limit (%.0f deg margin)', ...
%     margin_deg)});
xlim([-5 20]);
legend('Location','best');
set(gca,'FontSize',11);

% Retained in workspace: results, alpha_deg, CL_curve, CL_required.
% No post-stall curve is invented. CL_curve uses NaN beyond modeled stall.
% A high CL alone does NOT demonstrate approach or field-length feasibility:
% check net force, trim/control, failures, and takeoff/landing dynamics next.
% The effective CL includes the force convention of the slides; do not add
% propeller normal force again. Do not multiply by (Vj/V)^2 again either.

%% LOCAL FUNCTIONS
function increment = poweredIncrement(alpha, deltaJ, deltaCJ)
% Appendix A1: F(alpha,deltaJ,deltaCJ) - F(alpha,deltaJ,0).
% All angles are numeric degrees. A0 cancels in the subtraction.
    A1 = 0.0121*deltaJ - 0.0085*alpha + 0.0426;
    A2 = 0.0140*deltaJ + 0.0226*alpha + 0.1407;
    increment = A1.*sqrt(deltaCJ) + A2.*deltaCJ;
end

function eta = turningEfficiency(sigma, flap)
% Appendix A5 digitized curves; no extrapolation beyond sigma in [0,1].
    sigma40 = [0 .29 .35 .40 .459 .518 .581 .75 .859 1];
    eta40   = [1 1 .988 .970 .940 .900 .846 .619 .449 .253];
    sigma60 = [0 .29 .38 .429 .519 .548 .581 .75 .859 1];
    eta60   = [1 .869 .813 .781 .713 .686 .655 .384 .176 .041];
    e40 = interp1(sigma40,eta40,sigma,'linear');
    e60 = interp1(sigma60,eta60,sigma,'linear');
    blend = min(max((flap-40)/20,0),1);
    eta = (1-blend)*e40 + blend*e60;
end
