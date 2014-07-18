function Hd = LPF2
%LPF2 Returns a discrete-time filter object.

% MATLAB Code
% Generated by MATLAB(R) 8.2 and the DSP System Toolbox 8.5.
% Generated on: 24-Jun-2014 01:09:51

% Equiripple Lowpass filter designed using the FIRPM function.

% All frequency values are in Hz.
Fs = 8000000;  % Sampling Frequency

Fpass = 3000000;         % Passband Frequency
Fstop = 4000000;         % Stopband Frequency
Dpass = 0.057501127785;  % Passband Ripple
Dstop = 0.01;            % Stopband Attenuation
dens  = 20;              % Density Factor

% Calculate the order from the parameters using FIRPMORD.
[N, Fo, Ao, W] = firpmord([Fpass, Fstop]/(Fs/2), [1 0], [Dpass, Dstop]);

% Calculate the coefficients using the FIRPM function.
b  = firpm(N, Fo, Ao, W, {dens});
Hd = dsp.FIRFilter( ...
    'Numerator', b);

% [EOF]