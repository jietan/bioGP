% clear all;
nEpisodes = 2; 
% 128 64 32

degToRad = 2 * pi / 360;
a = [3.16e04; 0; -1.25e06] * degToRad;
b = [-2.047e04; -2.749e04; 4.646e04] * degToRad;
c = [5304; 7992; 1.026e04] * degToRad;
d = [-677.8; -675.8; -681.3] * degToRad;
e = [37.42; 18.3; 9.368] * degToRad;
endTime = [0.2; 0.06; 0.035];
weight = [1 1 1];
power = [1 2 4];
% weight = 1 ./ endTime;
count = 0;

stepSize = 0.001;

episodeSize = endTime / stepSize;
cumEpisodeSize = cumsum(episodeSize);

for i = 1 : nEpisodes  
    for ithSample = 1 : episodeSize(i);
        count = count + 1;
        t = ithSample * stepSize;
        x(count) = a(i) * t^4 + b(i) * t^3 + c(i) * t^2 + d(i) * t + e(i);
        xdot(count) = -(4 * a(i) * t^3 + 3 * b(i) * t^2 + 2 * c(i) * t + d(i));
        xddot(count) = -(12 * a(i) * t^2 + 6 * b(i) * t + 2 * c(i));
    end
end

A = zeros([count, 3]);
b = zeros([count, 1]);
i = 0;
for ithEpisode = 1 : nEpisodes
    for ithSample = 1 : episodeSize(ithEpisode)
        if ithEpisode == 1
            i = ithSample;
        else
            i = cumEpisodeSize(ithEpisode - 1) + ithSample;
        end
        A(i, 1) = weight(ithEpisode) * 1;
        A(i, 2) = weight(ithEpisode) * xdot(i);
        A(i, 3) = weight(ithEpisode) * power(ithEpisode) * x(i);
        b(i) = weight(ithEpisode) * xddot(i);
    end
end
% for i = 1 : count
%     A(i, 1) = 1;
%     A(i, 2) = xdot(i);
%     A(i, 3) = (i <= episodeSize(1)) * x(i);
%     for j = 2 : nEpisodes
%         ithSampleInEpisode = (i - episodeSize(j - 1));
%         A(i, 2 + j) = (ithSampleInEpisode > 0) * (ithSampleInEpisode <= episodeSize(j)) * x(i);
%     end
%     b(i) = xddot(i);
% end
params = A \ b
params(3) * power(2)
params(3) * power(3)
