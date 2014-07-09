addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_filter/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/subsample/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_interp/');

data0 = csvread('0data.csv');
%data0 = data0 - 128; %remove 1.5V offset
ave = mean(data0); 
data0 = data0 - ave;
ave;
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
figure('Name','seqb');
subplot(1,1,1);
pwelch(seqb);
start = .1875;
seqb = sinc_filter(seqb, [start start+0.125]);
figure('Name','seqb');
subplot(1,1,1);
pwelch(seqb);
endpt = 3000;
split = 20;
samp = 24E6;
xl = length(sinc_interp(seqb(1:endpt),0:1/(samp):(endpt-1)/(samp),0:1/(split*samp):(endpt-1)/(split*samp)));
out = zeros(xl,1);
out = sinc_interp(seqb(1:endpt),0:1/(samp):(endpt-1)/(samp),0:1/(split*samp):(endpt-1)/(split*samp));

%seqb = filter(Hd,seqb);
%seqb = filter(Hd2,seqb);filter(Hd_3,
%
%seqb = filter(Hd_3,seqb);
%seqb = filter(Hd_test,seqb);

len = 400;
offset = 0; %4000;

figure('Name', 'Raw Data')

 for n = 1:gels+1
    if n ~= gels+1
        
        subplot(gels+1,1,n);
        plot(p(1:len,n));
        ylim([-75 75]); 
       

    else
        subplot(gels+1,1,n);
        plot(out);
        ylim([-75 75]);

        
    end
    
end
        
figure('Name', 'Display Results')
subplot(2,1,1);
pwelch(out(1:end));
subplot(2,1,2);
plot(out(length(out)/2:length(out)/2+length(out)/4));
r = snr(out(1:end))
