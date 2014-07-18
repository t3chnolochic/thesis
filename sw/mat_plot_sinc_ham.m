addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_filter/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/subsample/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_interp/');

samp = 3E6;

data0 = csvread('0data.csv');
%remove 1.5V offset
ave = mean(data0); 
data0 = data0 - ave;

tot = 16; %7*3;
gels = 8;
hc = (tot)/(2*gels);
buff = 75;
pn = zeros(62*buff,gels);

ttime = (1/samp)*(length(data0(1:62*buff)) -1)
kern = 101;%length of filter
M = kern;
H = zeros(kern,1); %filter kernel
FC = 0.5; %half cut off freq
p = zeros(62*buff,gels);

ps = zeros(4650+kern-1,1);

for k = 1:kern
    if ((k-M/2) == 0) 
        H(k) = 2*pi*FC;
    else
    H(k) = sin(2*pi*FC * (k-M/2)) / (k-M/2); 
    H(k) = H(k) * (0.54 - 0.46*cos(2*pi*k/M)); %Sinc with Hamming window
    end
    
end

H = H/sum(H);

% figure('Name','H = SINC+HAM');
% subplot(1,1,1);
% plot(H);


for n = 0:(gels-1)
    
    
    p(:,(n+1)) = data0((1+n*62*buff):62*buff*(1+n));
    display('Convolving');
    %ps(:,(n+1)) = conv(p(1:100,n+1),H);
    ps(:,(n+1)) = conv(p(:,n+1),H);
    %pn(:,(n+1)) = sinc_filter(data0((1+n*62*buff):62*buff*(1+n)),1);
    
end



seqa = [];
seqb = [];


for ii = kern:length(ps)-kern;
    
    for i = 1:gels
        seqb(end+1) = ps(ii,i);
    end
    
       
end

figure('Name','seqb');
subplot(1,1,1);
pwelch(seqb);

start = .87083;
seqb = sinc_filter(seqb, [start start+0.125]); %MOVING WINDOW DEPENDS ON FREQUENCY OF INTEREST

figure('Name','seqb');
subplot(1,1,1);
pwelch(seqb);

endpt = 3000;
split = 25;
samp = 24E6;
xl = length(sinc_interp(seqb(1:endpt),0:1/(samp):(endpt-1)/(samp),0:1/(split*samp):(endpt-1)/(split*samp)));
out = zeros(xl,1);
out = sinc_interp(seqb(1:endpt),0:1/(samp):(endpt-1)/(samp),0:1/(split*samp):(endpt-1)/(split*samp));

len = kern+600;
offset = 0; %4000;


figure('Name', 'Raw Data')

 for n = 1:gels+1
    if n ~= gels+1
        
        subplot(gels+1,1,n);
        plot(ps(kern:len,n));
        ylim([-75 75]); 

    else
        subplot(gels+1,1,n);
        %plot(out((1+offset):(len*(n-1)+offset)));
        plot(out);
        ylim([-75 75]);

    end
    
 end
        
offset2 = 0;
figure('Name', 'Display Results')
subplot(2,1,1);
pwelch(out(1:end));
subplot(2,1,2);
%plot(seqb((1+offset2):(offset2+len)));
plot(out(length(out)/2:end))
r = snr(out(1:end))


figure('Name', 'END')
plot(out(length(out)/2:end))



