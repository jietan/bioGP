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
    angle = sqrt(sum(abs(rotData).^2,2));
    plot(t, angle, 'r');
%     angle = data(:, 12);
%     plot(t, angle, 'r');
%     angle = data(:, 14);
%     plot(t, angle, 'r');
end

% sim data
data = dlmread('C:\Users\JieTan\Documents\MyProjects\bioloid\bioloidgp-master\data\recording\lean-to-stand-0.13.sim', ' ',1,0);
hold all;
t = data(:,1);
rotData = data(:,2:4);
angle = sqrt(sum(abs(rotData).^2,2));
plot(t, angle, 'b');
%     angle = data(:, 12);
%     plot(t, angle, 'b');
%     angle = data(:, 14);
%     plot(t, angle, 'b');
    