addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_filter/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/subsample/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_interp/');

data0 = csvread('0data.csv');
%data0 = data0 - 128; %remove 1.5V offset
trigger_set = data0(1:62);
mean(trigger_set(2:8))
data0 = data0(63:end);
%ave = mean(data0); 
ave = 136; %1.6V/3.0V * 256
data0 = data0 - ave;

trigger = trigger_set(2); 
trigger = trigger - ave;

r = snr(data0);

figure('Name','Normal - No Seq');
subplot(3,1,1);
plot(1:length(data0)/100,data0(1:length(data0)/100),1:length(data0)/100,trigger);
title('Raw Data from ADC')
xlabel('1/3E6, samples')
ylabel('0-3V, 0 - 255')

seqraw = data0';

endpt = 2000;
split = 2;
samp = 3E6;

xl = length(sinc_interp(seqraw(1:endpt),0:1/(samp):(endpt-1)/(samp),0:1/(split*samp):(2*endpt-1)/(split*samp)));
out = zeros(xl,1);
out = sinc_interp(seqraw(1:endpt),0:1/(samp):(endpt-1)/(samp),0:1/(split*samp):(2*endpt-1)/(split*samp));
seqb2 = out;
length(seqb2);

subplot(3,1,2);
plot(out(1:length(out)/5));
title('Interpolated by 2')
ylabel('0-3V, 0 - 255')

samp1 = 6E6;

NFFT = 2^nextpow2(length(out));
Y = fft(out,NFFT)/length(out);
%plot(fft(out));
f = samp1/2*linspace(0,1,NFFT/2+1);
f = f(1:end/2);
%Plot single-sided amplitude spectrum.
new = 2*abs(Y(1:NFFT/2+1));
new = new(1:end/2);

subplot(3,1,3);
%plot(f,new);
semilogx(f,new);
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')
