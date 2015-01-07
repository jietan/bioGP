clear all;
nEpisodes = 4; 
% 128 right 64 right 128 left 64 left


degToRad = 2 * pi / 360;
% a = [-1.473e05; -2808] * degToRad;
% b = [4.794e04; -1.399e04] * degToRad;
% c = [-3728; 5783] * degToRad;
% d = [-281.9; -669.8] * degToRad;
% e = [36.71; 19.13] * degToRad;
% endTime = [0.13; 0.08];

% a = [-3.84e05; -9.411e05] * degToRad;
% b = [7.012e04; 1.084e05] * degToRad;
% c = [-1035; 1151] * degToRad;
% d = [-590.2; -624.8] * degToRad;
% e = [37.34; 18.44] * degToRad;
% 
% endTime = [0.067; 0.06];

a = [3.254e04 ; 2.718e04 ; -3.64e04 ;  -1.688e04  ] * degToRad;
b = [-1480    ; 1.308e04 ; 3493     ;  -1.985e04  ] * degToRad;
c = [-3482    ; -6500    ; 3220     ;  7543       ] * degToRad;
d = [690.4    ; 705.7    ; -687.1   ;  -747.2     ] * degToRad;
e = [-36.84   ; -18.52   ; 37.84    ;  18.98      ] * degToRad;


endTime = [0.16; 0.16; 0.18; 0.15];
weight = [1 1 1 1];
power = [1 2 1 2];
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
