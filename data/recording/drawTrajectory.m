directory = 'C:\Users\JieTan\Documents\MyProjects\bioloid\bioloidgp-master\data\recording\lean-to-stand\';
% files = dir(strcat(directory, '0.1-*.measure'));
% for file = files'
%     data = dlmread(strcat(directory, file.name),' ',1,0);
%     hold all;
%     t = data(:,1);
%     rotData = data(:,2:4);
% %     angle = sqrt(sum(abs(rotData).^2,2));
%     angle = data(:, 12); % left thigh joint
%     plot(t, angle, 'b');
%     angle = data(:, 14); % right thigh joint
%     plot(t, angle, 'b');
% end
% 
files = dir(strcat(directory, '0.13-*.measure'));
for file = files'
    data = dlmread(strcat(directory, file.name),' ',1,0);
    hold all;
    t = data(:,1);
    rotData = data(:,2:4);
%     angle = sqrt(sum(abs(rotData).^2,2));
%     plot(t, angle, 'r');
    angle = data(:, 12);
    plot(t, angle, 'r');
    angle = data(:, 14);
    plot(t, angle, 'r');
end

% sim data
data = dlmread('C:\Users\JieTan\Documents\MyProjects\bioloid\bioloidgp-master\data\recording\lean-to-stand-0.13.sim', ' ',1,0);
hold all;
t = data(:,1);
rotData = data(:,2:4);
% angle = sqrt(sum(abs(rotData).^2,2));
% plot(t, angle, 'b');
    angle = data(:, 12);
    plot(t, angle, 'b');
    angle = data(:, 14);
    plot(t, angle, 'b');
    
% files = dir(strcat(directory, '0.13-*.measure'));
% resampledTime = 0:0.001:1.4;
% resampledData = zeros(size(resampledTime, 2), 22);
% nIterations = 0;
% for file = files'
%     data = dlmread(strcat(directory, file.name),' ',1,0);
%     hold all;
%     t = data(:,1);
%     ncols = size(data, 2);
%     dofs = data(:, 2:ncols-1);
%     ts = timeseries(dofs, t);
%     
%     resampledTs = resample(ts, resampledTime);
%     resampledData = resampledData + resampledTs.data;
%     nIterations = nIterations + 1;
% end
% resampledData = resampledData / nIterations;
% resampledData = [resampledTime' resampledData];
% save('C:\Users\JieTan\Documents\MyProjects\bioloid\bioloidgp-master\data\recording\lean-to-stand\0.13avg.measure', 'resampledData', '-ascii');



