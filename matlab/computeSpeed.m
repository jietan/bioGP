function speed = computeSpeed(p, t)
numSamples = size(p, 1);
filteredP = (p(1:numSamples-2) + p(2:numSamples-1) + p(3:numSamples)) / 3;
p(2:numSamples-1) = filteredP;
dt = t(2:numSamples) - t(1:numSamples-1);
speed = (p(2:numSamples) - p(1:numSamples-1)) ./ dt;


