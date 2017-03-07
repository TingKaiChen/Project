euld = [];
tdd = [];
for i=1:length(accumeul)
%     euld = [euld max(accumeul(i,:))-min(accumeul(i,:))];
%     tdd = [tdd max(accumtd(i,:))-min(accumtd(i,:))];
    euld = [euld accumeul(i,1)-accumeul(i,4)];
    tdd = [tdd accumtd(i,1)-accumtd(i,4)];
end
figure()
subplot(2,1,1);
plot(euld)
subplot(2,1,2);
plot(tdd)