% motor 2825
StallTorque = 270 * 1.38874;  % 270 kgf·mm ≈ 375 oz-in
StallCurrent  = 5500;   % mA
RatedVoltage  = 12;     % V
NoLoadCurrent = 150;    % mA
NoLoadSpeed   = 150;   % RPM

% Generate the plots
figure;
pololuMotorPlotGenAAMv2(StallTorque, StallCurrent, ...
    RatedVoltage, NoLoadCurrent, NoLoadSpeed);