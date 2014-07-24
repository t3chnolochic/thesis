addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_filter/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/subsample/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_interp/');

data0 = csvread('0data.csv');
%data0 = data0 - 128; %remove 1.5V offset
ave = mean(data0); 
data0 = data0 - ave;
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


%sincy = sinc_interp(data0(1:62*buff)',t1,t1a);
%sincy_len = length(sincy);

%p = zeros(sincy_len,gels);
preseq = zeros((62*buff - hc*(gels-1)), gels);



for n = 0:(gels-1)
    
    %y = filter(Hd,data);
    %length(data0((1+n*62*buff):62*buff*(1+n)))
    %p(:,(n+1)) = sinc_interp(data0((1+n*62*buff):62*buff*(1+n))',t1,t1a);
    %p(:,(n+1)) = sinc_filter(data0((1+n*62*buff):62*buff*(1+n)),1);
    p(:,(n+1)) = data0((1+n*62*buff):62*buff*(1+n));
    x = p(hc*(gels-n-1) + 1:(end -hc*(n)),(n+1));
    preseq(:,(n+1)) = x;
    
end



seqa = [];
seqb = [];

% for ii = 1:length(preseq)
%     
%     for i = 1:gels
%         seqa(end+1) = preseq(ii,i);
%     end
%        
% end
% seqa = filter(Hd,seqa);

%just P data not chopped
for ii = 1:length(p);
    
    for i = 1:gels
        seqb(end+1) = p(ii,i);
    end
    
       
end
%index freq/12
endpt = 3000;
split = 2;
samp = 24E6;

xl = length(sinc_interp(seqb(1:endpt),0:1/(samp):(endpt-1)/(samp),0:1/(split*samp):(endpt-1)/(split*samp)));
out = zeros(xl,1);
out = sinc_interp(seqb(1:endpt),0:1/(samp):(endpt-1)/(samp),0:1/(split*samp):(endpt-1)/(split*samp));
seqb2 = out;

samp1 = 48E6;

figure('Name','seqb');
subplot(2,1,1);

NFFT = 2^nextpow2(length(seqb2));
Y = fft(seqb2,NFFT)/length(seqb2);
plot(fft(seqb2));
f = samp1/2*linspace(0,1,NFFT/2+1);
f = f(1:end/2);
% Plot single-sided amplitude spectrum.
new = 2*abs(Y(1:NFFT/2+1));
new = new(1:end/2);
plot(f,new); 


%xlim([0 12E6]);
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')

[val, idx] = max(new)

max_frq = (f(idx))/1E6; %max freq
frq = max_frq -.75; %.75 = 12 (MHz)/(gels*2), gels = 8
start = frq/24 ; 
frac = 0.125/2 ;

if (0.5 - start) <= frac
    sinc_end = 0.5;

elseif ( start < 0)
    start = 0;
    sinc_end = start + frac;
else
    sinc_end = start + frac;
end

seqb = sinc_filter(seqb2, [start sinc_end]); %goes from 0 to 1
%seqb = seqb2; %no sinc-ing
subplot(2,1,2);

NFFT = 2^nextpow2(length(seqb));
Y = fft(seqb,NFFT)/length(seqb);
plot(fft(seqb));
f = samp1/2*linspace(0,1,NFFT/2+1);
% Plot single-sided amplitude spectrum.
plot(f,2*abs(Y(1:NFFT/2+1))) 

xlim([0 12E6]);
title('Single-Sided Amplitude Spectrum of y(t) (after Sinc Block)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')



%pwelch(seqb);
endpt = 3000;
split = 10;

samp = 48E6;

xl = length(sinc_interp(seqb(1:endpt),0:1/(samp):(endpt-1)/(samp),0:1/(split*samp):(endpt-1)/(split*samp)));
out = zeros(xl,1);
out = sinc_interp(seqb(1:endpt),0:1/(samp):(endpt-1)/(samp),0:1/(split*samp):(endpt-1)/(split*samp));

len = 400;
offset = 0; %4000;

figure('Name', 'Raw Data')

 for n = 1:gels+1
    if n ~= gels+1
        
        subplot(gels+2,1,n);
        plot(1:len,p(1:len,n),1:len,data0(1));
        title(strcat('Gel', int2str(n),' #NoFilter'));
        ylim([-75 75]); 
       

    else
        subplot(gels+2,1,n);
        plot(1:length(out),out,1:length(out),data0(1));
        title('#Sinc-terpolated')
        ylim([-75 75]);
        
        subplot(gels+2,1,n+1);
        plot(1:400,seqb(1:400),1:400,data0(1));
        title('Seqb')
        ylim([-75 75]);

        
    end
    
end
        
figure('Name', 'Display Results')
subplot(4,1,1);

NFFT = 2^nextpow2(length(out));
Y = fft(seqb,NFFT)/length(out);
plot(fft(out));
f = samp/2*linspace(0,1,NFFT/2+1);
% Plot single-sided amplitude spectrum.
plot(f,2*abs(Y(1:NFFT/2+1))) 
title('#Sinc #FFTYourLife');
xlim([0 12E6]);

%pwelch(out(1:end));
subplot(4,1,2);
r = length(out(length(out)/2:length(out)/2+length(out)/4));
%plot(1:r,out(length(out)/2:length(out)/2+length(out)/4),1:r,data0(1));
plot(out(length(out)/2:length(out)/2+length(out)/4))
title('#SincTheDayQuarter');
r = snr(out(1:end))
subplot(4,1,3);
plot(out);
title('#SincTheDay');
subplot(4,1,4);
plot(seqb)
title('#NoFilter')
