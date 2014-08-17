addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_filter/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/subsample/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_interp/');

data0 = csvread('0data.csv');
%data0 = data0 - 128; %remove 1.5V offset
trigger_set = data0(1:62);
mean(trigger_set(2:8))
data0 = data0(63:end);
ave = mean(data0); 
data0 = data0 - ave;
trigger = mean(trigger_set(2:8)); 
trigger = trigger - ave;

r = snr(data0);
%plot(data0);
%figure('Name', 'Display Results')
%subplot(2,1,1);
%pwelch(data0(6000:8000));
%subplot(2,1,2);
%plot(data0(6000:8000));
tot = 16; %7*3;
gels = 8;
hc = (tot)/(2*gels);
buff = 75;
p = zeros(62*buff,gels);
preseq = zeros((62*buff - hc*(gels-1)), gels);



for n = 0:(gels-1)
    
    p(:,(n+1)) = data0((1+n*62*buff):62*buff*(1+n));
    x = p(hc*(gels-n-1) + 1:(end -hc*(n)),(n+1));
    preseq(:,(n+1)) = x;
    
end



seqa = [];
seqb = [];
seqraw = [];

%just P data not chopped
for ii = 1:length(p);
    
    for i = 1:gels
        seqraw(end+1) = p(ii,i);
        
    end
    
       
end

N = length(seqraw);
L = 3;
M = N*L;
x = 0:L:L*N-1;
xi = 0:M-1;
out = interpft(seqraw,M);
%plot(x,seqraw,'green',xi,out,'blue')
%legend('Original data','Interpolated data')

N2 = length(out);
L2 = 1;
M2 = N2*L2;
x2 = 0:L2:L2*N2-1;
xi2 = 0:M2-1;
out = interpft(out,M2);
%plot(x,seqraw,'green',xi,out,'blue')
%legend('Original data','Interpolated data')


samp1 = L2*L*24E6;

NFFT = 2^nextpow2(length(out));
Y = fft(out,NFFT)/length(out);
%plot(fft(out));
f = samp1/2*linspace(0,1,NFFT/2+1);
f = f(1:end/2);
% Plot single-sided amplitude spectrum.
new = 2*abs(Y(1:NFFT/2+1));
new = new(1:end/2);

figure('Name','seqb');
subplot(2,1,1);
plot(f,new);
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')

subplot(2,1,2);
plot(x,seqraw,'blue',xi2,out,'green')
legend('Original data','Interpolated data')


