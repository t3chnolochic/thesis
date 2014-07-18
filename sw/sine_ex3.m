addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_filter/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/subsample/');

samp = 3E6;
gold = 0.5 * ( 1 + sqrt(5) )
len = 100E-6;
t1=[0:1/samp:len];
t2=[0:1/(100E6):len];
A=1;
freq = (samp/2)/(1.6*gold) %140kHz

Wn = 1; %(samp/2)/(2*pi);

%Sine
y=A*sin(2*pi*freq*t1); %sampled
y=sinc_filter(y,Wn);
z=A*sin(2*pi*freq*t2); %perfect
%yfilter = filter(Hd,y);
ysinc = sinc_filter(y,Wn);
%ysub = subsampling(y,5);

figure('Name', 'Display Results')
subplot(3,1,1)
h = plot(t1,y, t2,z,'--');
set(h(1),'linewidth',3);
set(h(2),'linewidth',0.5);
%xlim([400E-6 (400E-6 + 300E-6)]);
legend('Sampled Sine','Perfect Sine');

subplot(3,1,2)

plot(ysinc)

% h = plot(t1,ysinc,t2,z,'--');
% set(h(1),'linewidth',3);
% set(h(2),'linewidth',0.5);
% legend('Filtered','Perfect');


subplot(3,1,3)
NFFT = 2^nextpow2(length(y));
Y = fft(y,NFFT)/length(y);
plot(fft(ysinc));
f = samp/2*linspace(0,1,NFFT/2+1);
% Plot single-sided amplitude spectrum.
semilogy(f,2*abs(Y(1:NFFT/2+1))) 
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')

% h = plot(t1,ysinc,t2,z,'--');
% legend('Sinc-ed','Perfect');
% set(h(1),'linewidth',3);
% set(h(2),'linewidth',0.5);
% xlim([400E-6 (400E-6 + 300E-6)]);