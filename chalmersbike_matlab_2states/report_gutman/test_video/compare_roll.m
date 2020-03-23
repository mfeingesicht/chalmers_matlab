clear all
close all
clc

%% Import video
% videoFileReader = vision.VideoFileReader('20191211_112450_VID.mp4');
videoFileReader = vision.VideoFileReader('20191212_094339_VID.mp4');
videoPlayer = vision.VideoPlayer('Position',[100,100,680,520]);
objectFrame = videoFileReader();


%% Get points to track
% Read the first video frame, which contains the object, define the region
% objectRegion = [264,122,93,93];

% As an alternative, you can use the following commands to select the object region using a mouse. The object must occupy the majority of the region:
% figure; imshow(objectFrame);
% objectRegion=round(getPosition(imrect));

% Show initial frame with a red bounding box.
% objectImage = insertShape(objectFrame,'Rectangle',objectRegion,'Color','red');
% figure;
% imshow(objectImage);
% title('Red box shows object region');

% Detect interest points in the object region.
% points = detectMinEigenFeatures(rgb2gray(objectFrame),'ROI',objectRegion);


%% User-defined points
points = struct;

% 20191211_112450_VID
% points.Location = [546.8016 201.7951 ; 625 700];
% points_verticalLine = [132 1 ; 147 720];

% 20191212_094339_VID
points.Location = [886 175 ; 1042 1029];
points_verticalLine = [625 84 ; 643 250];


%% Display the detected points.
pointImage = insertMarker(objectFrame,points.Location,'+','Color','white');
figure;
imshow(pointImage);
title('Detected interest points');


%% Create a tracker object.
tracker = vision.PointTracker('MaxBidirectionalError',1);


%% Initialize the tracker.
initialize(tracker,points.Location,objectFrame);


%% Read, track, display points, and results in each video frame.
points_allFrames = [];
while ~isDone(videoFileReader)
      frame = videoFileReader();
      [points,validity] = tracker(frame);
      points_allFrames = cat(3,points_allFrames,points);
      out = insertMarker(frame,points(validity, :),'+');
      videoPlayer(out);
end


%% Release the video reader and player.
release(videoPlayer);
release(videoFileReader);


%% Get x-y trajectory of points
points_trajx = [];
points_trajy = [];
time_video = (0:(size(points_allFrames,3)-1))/videoFileReader.info.VideoFrameRate;
for i=1:size(points,1)
    points_trajx(:,i) = squeeze(points_allFrames(i,1,:));
    points_trajy(:,i) = squeeze(points_allFrames(i,2,:));
end
% figure;plot(points_trajx,points_trajy);


%% Compute roll
roll = -atan2((points_trajx(:,2)-points_trajx(:,1)),(points_trajy(:,2)-points_trajy(:,1))) * 180/pi; % [deg]
figure(1);hold on;plot(time_video,roll);
title('Roll from video and data');


%% Compute rollrate
rollrate = diff(roll)./diff(time_video');
figure(2);hold on;plot(time_video(1:end-1),rollrate);
title('Roll Rate from video and data');


%% Extract roll from data gathered on the bike
% Read data into a table
% Data_exp = readtable('20191211_112450_IMU.csv');
Data_exp = readtable('20191212_094339_IMU.csv');

% Extract from table of data
time = Data_exp.Var1;
phi_RollAccComp = Data_exp.Var2;
phi = Data_exp.Var3;
phidot = rad2deg(Data_exp.Var4);

figure(1);plot([0 ; time+5.35],[phi(1) ; phi]);
legend('Roll from video','Roll from data');

figure(2);plot([0 ; time+5.35],[phidot(1) ; phidot]);
legend('Roll Rate from video','Roll Rate from data');