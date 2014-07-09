addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_filter/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/subsample/');
addpath('/Users/jinstone/Desktop/stmsetup/thesis/sw/sinc_interp/');

samp = 3E6;

data0 = csvread('0data.csv');
%remove 1.5V offset
ave = mean(data0); 
data0 = data0 - ave;
ave;
r = snr(data0);
plot(data0);
%figure('Name', 'Display Results')
%subplot(2,1,1);
%pwelch(data0(6000:8000));
%subplot(2,1,2);
%plot(data0(6000:8000));
tot = 16; %7*3;
gels = 8;
hc = (tot)/(2*gels);
buff = 75;
pn = zeros(62*buff,gels);

ttime = (1/samp)*(length(data0(1:62*buff)) -1)
t1=[0:1/samp:ttime];
t1a=[0:1/(8*samp):ttime];

sincy = sinc_interp(data0(1:10)',t1(1:10),t1a);
sincy_len = length(sincy);
p = zeros(62*buff,gels);
ps = zeros(sincy_len,gels);
%ps = zeros(62E3,gels);
preseq = zeros((62*buff - hc*(gels-1)), gels);



for n = 0:(gels-1)
    
    %y = filter(Hd,data);
    %length(data0((1+n*62*buff):62*buff*(1+n)));
    
    p(:,(n+1)) = data0((1+n*62*buff):62*buff*(1+n));
    q = length(p(1:100,n+1))
    ps(:,(n+1)) = sinc_interp(p(1:100,n+1)',t1(1:100),t1a);
    %pn(:,(n+1)) = sinc_filter(data0((1+n*62*buff):62*buff*(1+n)),1);
    %x = p(hc*(gels-n-1) + 1:(end -hc*(n)),(n+1));
    %preseq(:,(n+1)) = x;
    
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

for ii = 1:length(ps);
    
    for i = 1:gels
        seqb(end+1) = ps(ii,i);
    end
    
       
end
start = .5;
seqb = sinc_filter(seqb, 1); %MOVING WINDOW DEPENDS ON FREQUENCY OF INTEREST

%seqb = filter(Hd,seqb);
%seqb = filter(Hd2,seqb);filter(Hd_3,
%
%seqb = filter(Hd_3,seqb);
%seqb = filter(Hd_test,seqb);

len = 400;
offset = 0; %4000;

% figure('Name', 'Sequential Sampling')
% for n = 1:gels+1
%     if n ~= gels+1
%         
%         subplot(gels+1,1,n);
%         plot(preseq(1:len,n));
%         ylim([-50 50]);
%         xlim([1 len]);
% 
% 
%     else
%         subplot(gels+1,1,n);
%         plot(seqa(start:len*(n-1)));
%         ylim([-50 50]);
%         xlim([1 len*(n-1)]);
% 
%     end
%     
% end

figure('Name', 'Raw Data')

 for n = 1:gels+1
    if n ~= gels+1
        
        subplot(gels+1,1,n);
        plot(ps(1:len,n));
        ylim([-75 75]); 
        xlim([1 len]);

    else
        subplot(gels+1,1,n);
        plot(seqb((1+offset):(len*(n-1)+offset)));
        ylim([-75 75]);
        xlim([1 len*(n-1)]);

    end
    
 end
        
offset2 = 1000;
figure('Name', 'Display Results')
subplot(2,1,1);
pwelch(seqb(1:end));
subplot(2,1,2);
plot(seqb(offset2:offset2+len));
r = snr(seqb(1:end))

figure('Name', 'END')
subplot(1,1,1);
plot(seqb(500:500+len));


