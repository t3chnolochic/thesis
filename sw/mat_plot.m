data0 = csvread('0data.csv');
%data0 = data0 - 128; %remove 1.5V offset
ave = mean(data0); 
data0 = data0 - ave;
ave
r = snr(data0)

%figure('Name', 'Display Results')
%subplot(2,1,1);
%pwelch(data0(6000:8000));
%subplot(2,1,2);
%plot(data0(6000:8000));
tot = 40; %7*3;
gels = 4;
buff = 100;
p = zeros(62*buff,gels);

count = (tot)/gels; 
hc = count/2;

preseq = zeros( (62*buff-(hc-1)), gels);

for n = 0:(gels-1)
    p(:,(n+1)) = data0((1+n*62*buff):62*buff*(1+n));
    count = (tot)/gels; 
    
    x = p((gels-n): (end - (n+1)) , (n+1));
    preseq(:,(n+1)) = x;
    
end



seqa = [];
seqb = [];

for ii = 1:(62*buff-tot*(gels-1)/gels)
    
    for i = 1:gels
        seqa(end+1) = preseq(ii,i);
    end
       
end


%just P data not chopped
for ii = 1:(62*buff-tot*(gels-1)/gels)
    
    for i = 1:gels
        seqb(end+1) = p(ii,i);
    end
       
end

figure('Name', 'Sequential Sampling')

for n = 1:gels+1
    if n ~= gels+1
        
        subplot(gels+1,1,n);
        %plot(preseq((6000:6500),n));
        plot(preseq(1:500,n));
        ylim([-50 50]);
        xlim([0 50]);


    else
        subplot(gels+1,1,n);
        plot(seqa(1:2000));
        ylim([-50 50]);
        xlim([0 200]);
    end
    
end

figure('Name', 'Raw Data')

 for n = 1:gels+1
    if n ~= gels+1
        
        subplot(gels+1,1,n);
        %plot(preseq((6000:6500),n));
        plot(p(1:500,n));
        ylim([-50 50]);
        xlim([0 50]);

    else
        subplot(gels+1,1,n);
        plot(seqb(1:2000));
        ylim([-50 50]);
        xlim([0 200]);
    end
    
end
        
% figure('Name', 'Display Results')
% subplot(2,1,1);
% pwelch(seq(6000:8000));
% subplot(2,1,2);
% plot(seq(6000:8000));
% r = snr(seq(6000:8000))
