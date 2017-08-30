%% lawnmower scan pattern with down looking camera
%% generates waypoints for rotors
%% optionally generates directly poses for blender (vision-only use, besides the name vi_imu_poses.csv)  

close all;
clear;

%% Polygon of area
filename_polygon = 'sample_polygon.txt';
polygon=load(filename_polygon);
figure(1), hold on,
plot(polygon(:,1),polygon(:,2),'blue')

%% Rotate the polygon: This should be automated at some point.
th=0.83*pi;
R = [cos(th), -sin(th), 0.0;
    sin(th), cos(th), 0.0;
    0.0, 0.0, 1.0];
polygons_rotated = [];

for i=1:size(polygon,1)
    p = R * [polygon(i,1:2)';1];
    polygons_rotated=[polygons_rotated; p(1:2)'];
end
polygon = polygons_rotated;

figure(2), hold on,
plot(polygon(:,1),polygon(:,2),'red')

%% Rectangle of scan area
min_x = min(polygon(:,1));
max_x = max(polygon(:,1));
min_y = min(polygon(:,2));
max_y = max(polygon(:,2));

x_scan = [min_x max_x, max_x, min_x, min_x];
y_scan = [min_y, min_y, max_y, max_y, min_y];
figure(2), hold on, plot(x_scan, y_scan, 'r--', 'LineWidth', 2);

width = abs(max_x-min_x);
height = abs(max_y-min_y);

% Distance between scanlines
a = 0.1;
axis([min_x-a*width, max_x+a*width, min_y-a*height, max_y+a*height])

%% Define the output waypoints.
wps = [];
% Needs to be a multiple of 2.
n_rows = 6;

% Start at top left corner
wp0 = [min_x, max_y];
wps = [wps; wp0];
down = height / (n_rows - 1);


for idx_row = 1 : n_rows/2
    wp1 = wp0 + [width, 0.0];
    wp2 = wp1 - [0.0, down];
    wp3 = wp2 - [width, 0.0];
    wp4 = wp3 - [0.0, down];
    if idx_row < n_rows/2
      wps = [wps; wp1; wp2; wp3; wp4];
    else 
      wps = [wps; wp1; wp2; wp3];
    end
    
    % For next round:
    wp0 = wp4;
end

num_wps = size(wps,1);
figure(2), hold on, plot(wps(:,1), wps(:,2),'k');

wps_rotated=[];
for i=1:size(wps,1)
    wp = R' * [wps(i,1:2)';1];
    wps_rotated=[wps_rotated; wp(1:2)'];
end
figure(1), plot(wps_rotated(:,1), wps_rotated(:,2),'k');

altitude = 50;
yaw = 0.0;
[wps_rotated, altitude*ones(size(wps_rotated,1),1), yaw*ones(size(wps_rotated,1),1)]

pathXY = wps_rotated;
num_cameras = 850;
stepLengths = sqrt(sum(diff(pathXY,[],1).^2,2));
stepLengths = [0; stepLengths]; % add the starting point
cumulativeLen = cumsum(stepLengths);
finalStepLocs = linspace(0,cumulativeLen(end), num_cameras);
finalPathXY = interp1(cumulativeLen, pathXY, finalStepLocs);

figure(1), plot(finalPathXY(:,1), finalPathXY(:,2),'kx');

% Some random time.
dt_ns = 5000000;
time = 33005000000;

% Down-looking camera.
q = [2.21825285708e-06,0.707109018084,-2.21824259824e-06,0.707104544275];

vi_imu_poses=[];
opt_poses=[];
image_step=10;
for i=1:size(finalPathXY,1)
    vi_imu_pose=[time,finalPathXY(i,1:2),altitude,q];
    vi_imu_poses=[vi_imu_poses;vi_imu_pose];
    opt_poses=[opt_poses;vi_imu_pose];
    time=time + dt_ns;
end

dlmwrite('vi_imu_poses.csv',vi_imu_poses,'precision','%.6f');

figure(3), hold on
plot3(opt_poses(:,2),opt_poses(:,3),opt_poses(:,4),'g--')

