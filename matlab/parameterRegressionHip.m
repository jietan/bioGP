clear all;
nEpisodes = 2; 
% 128 64 32

degToRad = 2 * pi / 360;
a = [-1.473e05; -2808] * degToRad;
b = [4.794e04; -1.399e04] * degToRad;
c = [-3728; 5783] * degToRad;
d = [-281.9; -669.8] * degToRad;
e = [36.71; 19.13] * degToRad;
endTime = [0.11; 0.08];
weight = [1 1];
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
        xtrdot(count) = -(24 * a(i) * t + 6 * b(i));
    end
end

A = zeros([count, 2 + nEpisodes]);
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
        A(i, 2 + ithEpisode) = weight(ithEpisode) * x(i);
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
