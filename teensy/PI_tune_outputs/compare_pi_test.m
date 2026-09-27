%% compare_pi_test.m
% Compares the REAL closed-loop wheel speed (from motor_pi_test.ino) with a SIMULATION
% of the same PI controller running on the identified motor model.
% If the two match, the model and the controller design are validated.

clear; close all; clc;

%% 1. Settings - fill these in for the wheel you tested
file   = "pi_test_r_20260917_190909.csv";   % from capture_motor_id(port, 'l', "pi_test")
wheel  = "R";                               % "L" or "R"
Ts     = 0.01;                              % sample time (s)
maxPct = 25;                                % actuator limit (%)

% Identified motor model for THIS wheel - copy the exact numbers analyse_motor_id.m printed
K   = 0.5531;     % (rad/s) per %
tau = 0.042;      % s
td  = 0.016;      % s   dead time
d   = 1.63;       % %   deadband

% Controller gains for THIS wheel (exactly what is on the Teensy)
Kp = 1.152035;        % % per (rad/s)
Ki = 27.308961;         % % per rad

%% 2. Load the measured data
T   = readtable(file);
t   = T.t_ms / 1000;
ref = T.("ref" + wheel);                                 % reference (rad/s)
cmd = T.("cmd" + wheel);                                 % PI output (%)
w   = T.("thdot" + wheel);   % speed computed on the Teensy = exactly what the PI used (rad/s)
N   = numel(t);

% Every row should be exactly Ts apart. Missing rows break both diff(counts) and the simulation.
gaps = find(diff(T.t_ms) ~= round(Ts*1000));
if ~isempty(gaps)
    warning('%d gap(s) in the log, first at t = %.2f s. Re-capture before trusting this.', ...
            numel(gaps), t(gaps(1)));
end

%% 3. Simulate the same controller on the model, using the same reference
% Same order as the Teensy: measure speed -> compute PI -> command acts on the next interval.
a  = exp(-Ts / tau);          % exact discretisation of 1/(tau s + 1)
nd = round(td / Ts);          % dead time in samples

wSim = zeros(N, 1);
uSim = zeros(N, 1);
x = 0;                        % model speed
I = 0;                        % integrator

for k = 2:N
    % --- motor model: driven by the command sent (1 + nd) samples ago ---
    uIn  = uSim(max(k - 1 - nd, 1));
    uEff = sign(uIn) * max(abs(uIn) - d, 0);   % deadband
    x    = a*x + (1 - a)*K*uEff;               % first-order lag
    wSim(k) = x;

    % --- PI controller, identical to updatePI() on the Teensy ---
    if ref(k) == 0
        I = 0;
        uSim(k) = 0;
    else
        e      = ref(k) - wSim(k);
        uUnsat = Kp*e + I + d*sign(ref(k));
        u      = min(max(uUnsat, -maxPct), maxPct);
        if u == uUnsat || sign(e) ~= sign(uUnsat)
            I = I + Ki*e*Ts;
        end
        uSim(k) = u;
    end
end

%% 4. Plot measured vs simulated
figure('Color', 'w');
subplot(2, 1, 1);
plot(t, ref, 'k--', t, w, 'Color', [0.2 0.5 1]); hold on
plot(t, wSim, 'r-', 'LineWidth', 1.2);
ylabel('\theta dot (rad/s)'); grid on
legend('reference', 'measured', 'simulated', 'Location', 'best');
title("Wheel " + wheel + ": closed-loop speed, measured vs simulated");

subplot(2, 1, 2);
plot(t, cmd, 'Color', [0.2 0.5 1]); hold on
plot(t, uSim, 'r-', 'LineWidth', 1.2);
yline([maxPct -maxPct], 'k:');
ylabel('command (%)'); xlabel('t (s)'); grid on
legend('measured', 'simulated', 'Location', 'best');

%% 5. Step metrics for every non-zero reference segment
starts = [1; find(diff(ref) ~= 0) + 1];
ends   = [starts(2:end) - 1; N];

results = [];
for i = 2:numel(starts)
    r0 = ref(starts(i) - 1);            % reference before the step
    r1 = ref(starts(i));                % reference after the step
    if r1 == 0, continue; end           % skip stops (output forced to neutral)

    idx     = starts(i):ends(i);
    tt      = t(idx) - t(idx(1));
    settled = idx(round(end/2):end);
    stepSz  = r1 - r0;

    [osM, tsM] = stepInfo(tt, w(idx),    r1, stepSz);
    [osS, tsS] = stepInfo(tt, wSim(idx), r1, stepSz);
    essM   = mean(w(settled)) - r1;     % steady-state error (measured)
    jitter = std(cmd(settled));         % command noise once settled

    results(end+1, :) = [r0 r1 osM osS tsM tsS essM jitter]; %#ok<SAGROW>
end

disp(array2table(results, 'VariableNames', ...
    {'ref_from', 'ref_to', 'overshoot_meas_pct', 'overshoot_sim_pct', ...
     'settle5_meas_s', 'settle5_sim_s', 'ss_error_meas', 'cmd_std_pct'}));

%% local function
function [overshoot, tSettle] = stepInfo(tt, y, rFinal, stepSz)
    % overshoot: how far past the new reference it goes, as % of the step size
    overshoot = max(0, max((y - rFinal) * sign(stepSz))) / abs(stepSz) * 100;
    % settling: last time it was outside a 5% band around the new reference
    out = find(abs(y - rFinal) > 0.05 * abs(stepSz), 1, 'last');
    if isempty(out),        tSettle = 0;
    elseif out == numel(y), tSettle = NaN;     % never settled
    else,                   tSettle = tt(out + 1);
    end
end