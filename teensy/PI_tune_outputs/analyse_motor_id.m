%% analyse_motor_id.m
% Identifies the "Motor model" block of the cascaded architecture (L08 slides 8 and 13):
%
%       Thetadot(s) / U(s) = K e^(-td s) / (tau s + 1)
%
%   Thetadot = wheel angular velocity (theta_l dot / theta_r dot), rad/s
%   U        = Sabertooth command, %.  In linear R/C mode motor voltage V ~ U/100 * Vbatt,
%              so the slide-8 form Thetadot/V has gain K_V = K * 100 / Vbatt.
%   plus a static deadband d (handled by feedforward so the linear model holds).
%
% Then designs the inner-loop "Motor control" block (PI, lambda tuning), simulates it on a
% nonlinear plant with K/tau mismatch, and relates the result to v and omega through
% the kinematic model:  v = r(thdot_l + thdot_r)/2,  omega = r(thdot_r - thdot_l)/(2b).
% No toolboxes required.

clear; close all; clc;

% ---------------- user settings ----------------
file          = "motor_id_r_20260917_185144.csv";   % output of capture_motor_id
Ts            = 0.010;            % must match TS_US on the Teensy
countsPerRev  = 64*70;          % counts per WHEEL rev, must match the Teensy
maxPct        = 25;               % must match MAX_PERCENT on the Teensy
Vbatt         = 12.0;             % battery voltage during the test (V) - measure it
r_wheel       = 0.144;            % wheel radius r (m)          - MEASURE
b_half        = 0.27/2;            % half wheel base b (m), 2b = wheel base - MEASURE
lambdaFactor  = 0.5;              % closed-loop time constant = lambdaFactor * tau
mismatch      = [0.7 1.0 1.3];    % multipliers on K and tau for robustness sims
% -----------------------------------------------

T = readtable(file);
t = T.t_ms / 1000;

% Recompute wheel angular velocity from raw counts (independent of the Teensy constant)
U = [T.cmdL, T.cmdR];
Y = [[0; diff(T.countL)], [0; diff(T.countR)]] * 2*pi / (countsPerRev * Ts);

res = 2*pi / (countsPerRev * Ts);
fprintf('Wheel speed resolution: %.3f rad/s per encoder count per sample\n', res);

wheelNames = ["Left", "Right"];
tag        = ["L", "R"];
id         = struct('K', {NaN, NaN}, 'd', {NaN, NaN}, 'tau', {NaN, NaN}, ...
                    'td', {NaN, NaN}, 'Kp', {NaN, NaN}, 'Ki', {NaN, NaN}, 'lambda', {NaN, NaN});

for w = 1:2
    u = U(:, w);
    y = Y(:, w);
    if all(u == 0), continue; end
    fprintf('\n=================== %s wheel ===================\n', wheelNames(w));

    %% ---- 1. Fit each step segment ----
    edges = [1; find(diff(u) ~= 0) + 1; numel(u) + 1];
    steps = [];   % columns: u_from u_to thdot_from thdot_ss tau td
    for i = 2:numel(edges) - 1
        pre  = max(edges(i-1), edges(i) - round(0.5/Ts)) : edges(i) - 1;
        idx  = edges(i) : edges(i+1) - 1;
        tt   = t(idx) - t(idx(1));
        yy   = y(idx);
        y0   = mean(y(pre));
        tail = idx(round(0.6 * numel(idx)) : end);
        yss  = mean(y(tail));
        row  = [u(edges(i) - 1), u(idx(1)), y0, yss, NaN, NaN];

        if abs(yss - y0) > 5 * res
            k63  = find(abs(yy - y0) >= 0.63 * abs(yss - y0), 1);
            p0   = [log(max(tt(k63), Ts)), 0.02];
            cost = @(p) sum((yy - foStep(tt, y0, yss, exp(p(1)), abs(p(2)))).^2);
            p    = fminsearch(cost, p0, optimset('Display', 'off'));
            row(5:6) = [exp(p(1)), abs(p(2))];
        end
        steps(end+1, :) = row; %#ok<SAGROW>
    end
    disp(array2table(steps, 'VariableNames', ...
        {'u_from', 'u_to', 'thdot_from', 'thdot_ss', 'tau_s', 'delay_s'}));

    %% ---- 2. Steady-state gain and deadband ----
    % thdot_ss = K*(u - d*sign(u))  ->  least squares on [u, -sign(u)]
    ss = steps(steps(:, 2) ~= 0 & abs(steps(:, 4)) > 3 * res, [2 4]);
    c  = [ss(:, 1), -sign(ss(:, 1))] \ ss(:, 2);
    K  = c(1);
    d  = max(c(2) / K, 0);

    good = ~isnan(steps(:, 5));
    tau  = median(steps(good, 5));
    td   = median(steps(good, 6));

    fprintf('K        = %.4f (rad/s) per %%   =  K_V = %.3f (rad/s) per V\n', K, K*100/Vbatt);
    fprintf('deadband = %.2f %%\n', d);
    fprintf('tau      = %.3f s   (range %.3f to %.3f s)\n', tau, min(steps(good, 5)), max(steps(good, 5)));
    fprintf('delay    = %.3f s\n', td);

    ym     = simOpenLoop(u, K, tau, td, d, Ts);
    fitPct = 100 * (1 - norm(y - ym) / norm(y - mean(y)));
    fprintf('Model fit over the full run: %.1f %%\n', fitPct);

    %% ---- 3. Inner-loop PI design (lambda tuning, pole-zero cancellation) ----
    lambda = max([lambdaFactor * tau, 2 * td, 5 * Ts]);
    Kp     = tau / (K * (lambda + td));
    Ki     = Kp / tau;

    fprintf('\nPI design: closed-loop time constant ~ %.3f s\n', lambda);
    fprintf('  Kp = %.5f %% per (rad/s),  Ki = %.5f %% per rad\n', Kp, Ki);
    fprintf('  Command jitter from one encoder count: %.2f %%\n', Kp * res);

    %% ---- 4. Robustness simulation on the nonlinear plant ----
    rRef = 0.5 * K * (maxPct - d);
    N    = round(max(6 * tau, 2) / Ts);
    tc   = (0:N-1)' * Ts;

    figure('Name', wheelNames(w) + " wheel", 'Color', 'w');

    subplot(3, 1, 1);
    yyaxis left
    plot(t, y, '-', 'Color', [0.65 0.65 0.65]); hold on
    plot(t, ym, 'b-', 'LineWidth', 1.2); ylabel('\theta dot (rad/s)');
    yyaxis right
    stairs(t, u, 'r-'); ylabel('command (%)');
    xlabel('t (s)'); legend('measured', 'model', 'command', 'Location', 'best');
    title(sprintf('%s wheel: measured vs identified model (fit %.0f%%)', wheelNames(w), fitPct));

    subplot(3, 1, 2);
    plot(ss(:, 1), ss(:, 2), 'ko', 'MarkerFaceColor', 'k'); hold on
    uu = linspace(-maxPct, maxPct, 400);
    plot(uu, K * sign(uu) .* max(abs(uu) - d, 0), 'b-', 'LineWidth', 1.2);
    grid on; xlabel('command (%)'); ylabel('steady-state \theta dot (rad/s)');
    title(sprintf('Static map: K = %.3f (rad/s)/%%, deadband = %.1f%%', K, d));

    subplot(3, 1, 3); hold on; grid on
    fprintf('\n  K x   tau x   overshoot   settle 2%%\n');
    for kK = mismatch
        for kT = mismatch
            [yMeas, ~, yTrue] = simClosedLoop(rRef, N, K*kK, tau*kT, td, d, Ts, ...
                                              Kp, Ki, d, maxPct, countsPerRev);
            [os, tsettle] = stepMetrics(tc, yTrue, rRef);
            fprintf('  %.1f   %.1f     %5.1f %%     %.3f s\n', kK, kT, os, tsettle);
            if kK == 1 && kT == 1
                plot(tc, yMeas, '-', 'Color', [0.7 0.8 1]);
                plot(tc, yTrue, 'b-', 'LineWidth', 2);
            else
                plot(tc, yTrue, '-', 'Color', [0.5 0.5 0.5]);
            end
        end
    end
    yline(rRef, 'r--');
    xlabel('t (s)'); ylabel('\theta dot (rad/s)');
    title('Closed-loop step: nominal (blue), encoder reading (light blue), K/\tau mismatch (grey)');

    id(w) = struct('K', K, 'd', d, 'tau', tau, 'td', td, 'Kp', Kp, 'Ki', Ki, 'lambda', lambda);

    fprintf('\n// ---- Teensy constants (%s wheel) ----\n', wheelNames(w));
    fprintf('constexpr float KP_%s       = %.6ff;  // %% per (rad/s)\n', tag(w), Kp);
    fprintf('constexpr float KI_%s       = %.6ff;  // %% per rad\n', tag(w), Ki);
    fprintf('constexpr float DEADBAND_%s = %.3ff;  // %% feedforward\n', tag(w), d);
end

%% ---- 5. Link to the kinematic model and the outer loop ----
if all(~isnan([id.K]))
    thdotMax = min([id(1).K * (maxPct - id(1).d), id(2).K * (maxPct - id(2).d)]);
    fprintf('\n=================== Kinematics ===================\n');
    fprintf('Inverse kinematics for the inner-loop references (outer loop -> inner loop):\n');
    fprintf('  thdot_l_ref = (v - b*omega) / r\n  thdot_r_ref = (v + b*omega) / r\n');
    fprintf('Usable wheel speed (slower wheel, at MAX_PERCENT): %.2f rad/s\n', thdotMax);
    fprintf('  -> max straight-line v   = r*thdot_max   = %.3f m/s\n', r_wheel * thdotMax);
    fprintf('  -> max spin-on-spot omega = r*thdot_max/b = %.3f rad/s\n', r_wheel * thdotMax / b_half);
    fprintf('  Keep |v|/r + b|omega|/r below this or the wheel saturates and the path bends.\n');
    fprintf('Open-loop K mismatch L vs R: %.1f %% (the inner loop removes this; without it\n', ...
            100 * abs(id(1).K - id(2).K) / mean([id.K]));
    fprintf('  the robot curves when commanded straight).\n');
    slowest = max([id.lambda] + [id.td]);
    fprintf('Slowest inner loop ~ %.3f s -> outer (pose) loop time constant >= %.2f s (5x rule, slide 12)\n', ...
            slowest, 5 * slowest);
end

%% ======================= local functions =======================

function ym = foStep(tt, y0, yss, tau, td)
    ym = y0 + (yss - y0) * (1 - exp(-max(tt - td, 0) / tau));
end

function ym = simOpenLoop(u, K, tau, td, d, Ts)
    a  = exp(-Ts / tau);
    nd = max(round(td / Ts), 0);
    ue = sign(u) .* max(abs(u) - d, 0);
    ue = [zeros(nd, 1); ue(1:end-nd)];
    ym = zeros(size(u));
    for k = 2:numel(u)
        ym(k) = a * ym(k-1) + (1 - a) * K * ue(k);
    end
end

function [yMeas, uCmd, yTrue] = simClosedLoop(r, N, K, tau, td, dPlant, Ts, ...
                                              Kp, Ki, dFF, maxPct, cpr)
    a     = exp(-Ts / tau);
    nd    = max(round(td / Ts), 0);
    holdN = max(1, round(0.020 / Ts));   % Servo library refreshes the pulse at 50 Hz
    ubuf  = zeros(nd + 1, 1);
    yTrue = zeros(N, 1); yMeas = zeros(N, 1); uCmd = zeros(N, 1);
    x = 0; I = 0; ang = 0; cntPrev = 0; uHeld = 0;

    for k = 2:N
        % plant: deadband -> first-order lag, driven by delayed command
        ud = ubuf(end);
        ue = sign(ud) * max(abs(ud) - dPlant, 0);
        x  = a * x + (1 - a) * K * ue;
        yTrue(k) = x;

        % encoder: integer counts per sample
        ang = ang + x * Ts;
        cnt = floor(ang / (2*pi) * cpr);
        yMeas(k) = (cnt - cntPrev) * 2*pi / (cpr * Ts);
        cntPrev  = cnt;

        % discrete PI with deadband feedforward and anti-windup
        e      = r - yMeas(k);
        uUnsat = Kp * e + I + dFF * sign(r);
        uc     = min(max(uUnsat, -maxPct), maxPct);
        if uc == uUnsat || sign(e) ~= sign(uUnsat)
            I = I + Ki * e * Ts;
        end
        uc = round(uc / 0.2) * 0.2;      % 1 us servo pulse = 0.2 %
        uCmd(k) = uc;

        if mod(k, holdN) == 0, uHeld = uc; end
        ubuf = [uHeld; ubuf(1:end-1)];
    end
end

function [os, tsettle] = stepMetrics(t, y, r)
    os  = max(0, (max(y) - r) / abs(r) * 100);
    out = find(abs(y - r) > 0.02 * abs(r), 1, 'last');
    if isempty(out),        tsettle = 0;
    elseif out == numel(y), tsettle = NaN;
    else,                   tsettle = t(out + 1);
    end
end
