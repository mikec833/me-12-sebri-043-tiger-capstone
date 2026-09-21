function outfile = capture_motor_id(port, wheel, prefix)
% CAPTURE_MOTOR_ID  Run the Teensy step test and save the log as CSV.
%
%   capture_motor_id(port, 'l')               % identification run  -> motor_id_l_....csv
%   capture_motor_id(port, 'l', "pi_test")    % closed-loop PI run  -> pi_test_l_....csv
%   'l' = left, 'r' = right, 'b' = both
%
%   Find the port with:  serialportlist("available")
%   Close the Arduino serial monitor first - only one program can hold the port.
%   Ctrl-C (or any error) sends an abort and closes the port -> motors neutral.

    if nargin < 2, wheel = 'b'; end
    if nargin < 3, prefix = "motor_id"; end

    s = serialport(port, 115200, "Timeout", 10);
    configureTerminator(s, "CR/LF");
    cleanup = onCleanup(@() stopAndClose(s)); %#ok<NASGU>

    pause(0.5);
    flush(s);
    write(s, wheel, "char");
    fprintf('Running test on wheel ''%s''. Ctrl-C aborts.\n', wheel);

    % Read in bulk rather than line by line: readline() in a loop is too slow at
    % 100 lines/s, and when MATLAB falls behind the Teensy drops USB data.
    lines   = strings(0, 1);
    buf     = "";
    lastRx  = tic;
    done    = false;
    while ~done
        n = s.NumBytesAvailable;
        if n == 0
            if toc(lastRx) > 10
                error('Timed out waiting for the Teensy.');
            end
            pause(0.005);
            continue
        end
        lastRx = tic;
        buf    = buf + read(s, n, "string");
        parts  = split(buf, newline);
        buf    = parts(end);                       % keep any incomplete last line
        parts  = strtrim(parts(1:end-1));

        if any(startsWith(parts, "#ABORT"))
            warning('Teensy aborted: %s', parts(find(startsWith(parts, "#ABORT"), 1)));
            done = true;
        end
        if any(startsWith(parts, "#END"))
            done = true;
        end
        keep  = parts(parts ~= "" & ~startsWith(parts, "#"));
        lines = [lines; keep]; %#ok<AGROW>
        fprintf('.');
    end
    fprintf('\n');

    % ---- sanity checks: header present and no missing rows ----
    if isempty(lines) || ~startsWith(lines(1), "t_ms")
        warning(['No header line - data was lost at the start. Is another program ' ...
                 '(Arduino serial monitor/plotter) reading the same port?']);
    end
    tms  = str2double(extractBefore(lines(startsWith(lines, digitsPattern)), ","));
    nGap = sum(diff(tms) ~= median(diff(tms)));
    if nGap > 0
        warning('%d gap(s) in the log - rows were lost. Close anything else using the port and re-capture.', nGap);
    else
        fprintf('Log check OK: header present, no missing rows.\n');
    end

    outfile = sprintf('%s_%s_%s.csv', prefix, wheel, datestr(now, 'yyyymmdd_HHMMSS'));
    fid = fopen(outfile, 'w');
    fprintf(fid, '%s\n', lines);
    fclose(fid);
    fprintf('Saved %d samples to %s\n', numel(lines) - 1, outfile);
end

function stopAndClose(s)
    try
        write(s, 'x', "char");   % harmless if the run already finished
    catch
    end
    delete(s);                   % dropping DTR also makes the Teensy go neutral
end