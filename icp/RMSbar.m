function RMSbar(RMS)
    num = zeros(size(RMS, 1),10);
    for i=1:size(RMS,1)
        for j=1:10
            num(i,j) = sum(RMS(i,:)<(j/10))-sum(RMS(i,:)<((j-1)/10));
        end
    end
    figure('Name','RMS distribution')
    bar(num')
end