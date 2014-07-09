addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_filter/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_interp/');

samp = 437E3;
gold = 0.5 * ( 1 + sqrt(5) )
len = 10000E-6;
t1=[0:1/samp:len];
t1a=[0:1/(5*samp):len];
t2=[0:1/(100E6):len];
A=1;
freq = (samp/2)/(gold) %140kHz
Wn = 1; %(samp/2)/(2*pi);

%Sine
y=A*sin(2*pi*freq*t1); %sampled
z=A*sin(2*pi*freq*t2); %perfect

% ysinc1 = zeros(length(y));
% for i = 1:length(y)
%     for j = 1:length(y)
%         ysinc1(i) = ysinc1(i) + y(j)*sinc(i-j);
%     end
%     
% end

ysinc1 = sinc_interp(y,t1,t1a);
%ysinc1 = sinc(y);
ysinc2 = 0; %y; %sinc_filter(y,[0 1]);


%Sawtooth
ysaw = A*sawtooth(2*pi*freq*t1);
ysaw = sinc_filter(ysaw,Wn);
zsaw = A*sawtooth(2*pi*freq*t2);

figure('Name', 'Display Results')
subplot(3,1,1)
h = plot(t1,y, t2,z,'--');
set(h(1),'linewidth',0.5);
set(h(2),'linewidth',0.5);
%xlim([400E-6 (400E-6 + 300E-6)]);
legend('Sampled Sine','Perfect Sine');

subplot(3,1,2)
h = plot(1:length(ysinc1),ysinc1,1:length(ysinc2),ysinc2);
%xlim([1500 1600]);
set(h(1),'linewidth',0.5);
set(h(2),'linewidth',0.5);
legend('ysinc1','ysinc2');
 
% h = plot(t1,ysinc,t2,z,'--');
% set(h(1),'linewidth',3);
% set(h(2),'linewidth',0.5);
% legend('Filtered','Perfect');


% snr(z)
% snr(ysinc)
% snr(yfilter)

% xlim([400E-6 (400E-6 + 300E-6)]);
% h2 = plot(t1,ysaw,t2,zsaw,'--')
% set(h2(1),'linewidth',3);
% set(h2(2),'linewidth',0.5);
% xlim([0 300E-6]);
% legend('Sampled Saw','Perfect Saw');

subplot(3,1,3)
NFFT = 2^nextpow2(length(y));
Y = fft(y,NFFT)/length(y);
plot(fft(ysinc1));
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

