%% Load data file
load map_v1.mat;            % Map data (.mat format) taken as the OUTPUT of the "drivingScenarioDesigner App"

% Original map waypoints to put the converted waypoints from OPENDrive into the right order of sequence in MOBATSim
load waypoints_origin.mat;  % (Important to keep the crossroad waypoints correct)

%% Initialization
waypoints_new = [];% waypoints of the extended map before the right sequence is ordered
connections_circle = [];
connections_translation = [];
waypoints = [];%waypoints of extended map with the right order
WP_from_app = []; % waypoints got from drivingScenarioDesigner
Route_LaneNumber = []; % [Route_id, LaneNumber]


%% waypoints
% Get all the waypoints from drivingScenarioDesigner output file
Start = data.RoadSpecifications(1,1).Centers(1,:); % Start point of the first road section in drivingScenarioDesigner
End = data.RoadSpecifications(1,1).Centers(end,:); % End point of the first road section in drivingScenarioDesigner
WP_from_app = [Start,;End,];% Start and end point of the  first road section in drivingScenarioDesigner

% From the second road section, put all the start and end points together
for i = 2:size(data.RoadSpecifications,2)
    WP_from_app = [WP_from_app; 
                  data.RoadSpecifications(1,i).Centers(1,:);
                  data.RoadSpecifications(1,i).Centers(end,:)];   
end
B1 = unique(WP_from_app(:,1:3),'row','stable'); % remove all the repeated points with same lane number(coordinate in drivingScenarioDesigner)

%%Coordinate transformation
% Coordinate transformation(from drivingScenarioDesigner coordinat to MOBATSim coordimate)
% [y,-x,0]->[x,0,-y]
waypoints_new(:,1) = -B1(:,2);
waypoints_new(:,3) = -B1(:,1);
% Put the new waypoints in the same sequence as before
waypoints =  Order_sequence(waypoints_origin,waypoints_new);
B2 = [];
B2(:,1) = -waypoints(:,3);% after sequence order, transfer MOBATSim coordinate to drivingScenariodesigner coordinate
B2(:,2) = -waypoints(:,1);
B2(:,3) = 0;
 
Circle = zeros(1,7);
h = 1;
%% connection_transition
for j = 1:size(data.RoadSpecifications,2)
    if size(data.RoadSpecifications(1,j).Centers,1)==2 % if there are only 2 points in one road section, it is straight road
    a = ismember(B2, data.RoadSpecifications(1,j).Centers(1,:),'rows');
    m = find(a==1); %index of end point of transition
    b = ismember(B2, data.RoadSpecifications(1,j).Centers(2,:),'rows');
    n = find(b==1); % index of start point of transition
    connections_translation(h,:) = [n m 100];% find the sequence of translation connections
    h = h+1;
%% connection_circle
    elseif size(data.RoadSpecifications(1,j).Centers,1)>2 % if there are more than 2 points in one road section, it is arc
    % Find three points on tne arc
    C = data.RoadSpecifications(1,j).Centers(1,1:2); % only takes x and y for circle center calculation
    D = data.RoadSpecifications(1,j).Centers(2,1:2);
    E = data.RoadSpecifications(1,j).Centers(end,1:2);
    % Calculate the circle center 
    [x0, y0, r] = Circle_center(C,D,E);
    k = 1/r; % calculate curvature for the future speed limit

    F1 = data.RoadSpecifications(1,j).Centers(end,:);
    F2 = data.RoadSpecifications(1,j).Centers(1,:);
    a = ismember(B2, F1,'rows'); % find the sequence of circle connection
    m = find(a==1);
    b = ismember(B2, F2,'rows');
    n = find(b==1);
         
    % Polar coordinate
    F1 = F1(1:2);
    F2 = F2(1:2);
    G1 = F1-[x0,y0];% the coordinate of F1 relative to circle center
    G2 = F2-[x0,y0];
    [theta1, rho1] = cart2pol(G1(1),G1(2));% polar coordinate of G1
    [theta2, rho2] = cart2pol(G2(1),G2(2));
    if theta1 < 0 % [-pi~pi]->[0~2pi]
        theta1 = theta1+2*pi;     
    end
    if theta2 < 0
        theta2 = theta2+2*pi;
    end
    %disp([j-55, 0, theta1, theta2]);
    alpha = theta2-theta1; % caltulate the the angle of arc
    if alpha > pi % if arc accross the positive part of x axis
        alpha = alpha - 2*pi; % 
    elseif alpha < -pi
        alpha = alpha + 2*pi;
    elseif abs(alpha) == pi
        F3 = data.RoadSpecifications(1,j).Centers(2,:);
        F3 = F3(1:2);
        G3 = F3-[x0, y0];
        [theta3, rho3] = cart2pol(G3(1),G3(2));
        alpha1 = theta3-theta1; 
        alpha2 = theta2-theta3;%find another point between start point and end point, to judge the direction of the arc.
        if alpha1 > pi
            alpha1 = alpha1-2*pi;
        elseif alpha1 < -pi
            alpha1 = alpha1+2*pi;
        end
        if alpha2 > pi
            alpha2 = alpha2-2*pi;
        elseif alpha2 < -pi
            alpha2 = alpha2+2*pi;
        end
            alpha = alpha1+alpha2; %seperate arc in to two parts, and calculate the sum of two angle.
    end
    Circle(1,1) = m;
    Circle(1,2) = n;
    Circle(1,3) = alpha;
    Circle(1,4) = -y0;
    Circle(1,5) = 0;
    Circle(1,6) = -x0;
    Circle(1,7) = 100;
%     if k>0.0007 % Set speed limit depending on the curvature
%         Circle(1,7) = 50;
%     else
%         Circle(1,7) = 100;
%     end
   connections_circle = [connections_circle; Circle];   
   end    
end

%% start to find lane number for every connections
WP_from_app_LaneNum = [];%[start, end, laneNumber]
%get the waypoints and the coorsponding lane number from map data
for i = 1:size(data.RoadSpecifications,2)
    WP_from_app_LaneNum = [WP_from_app_LaneNum; 
                  data.RoadSpecifications(1,i).Centers(1,:),data.RoadSpecifications(1,i).Centers(end,:),data.RoadSpecifications(1,i).Lanes.NumLanes];   
end

%%coordinate transformation
% [y,-x,0]->[x,0,-y]
WP_from_app_LaneNum_new(:,1) = -WP_from_app_LaneNum(:,2);
WP_from_app_LaneNum_new(:,3) = -WP_from_app_LaneNum(:,1);
WP_from_app_LaneNum_new(:,4) = -WP_from_app_LaneNum(:,5);
WP_from_app_LaneNum_new(:,6) = -WP_from_app_LaneNum(:,4);
WP_from_app_LaneNum_new(:,7) =  WP_from_app_LaneNum(:,7);

%seperate the waypoints to startpoints and endpoints
startpoints = WP_from_app_LaneNum_new(:,4:6);
endpoints = WP_from_app_LaneNum_new(:,1:3);
 
circle_lane = [];%the lane number of all the circle
translation_lane = [];% the lane number of all the straight road
Index_circle = [];
Index_translation = [];

%%find the the lane number for coorsponding route
% curved road
 for i = 1:size(connections_circle,1)
     Index_circle = find(ismember(startpoints, waypoints(connections_circle(i,1),:),'rows'));
     for j = 1:size(Index_circle,1)
         if waypoints(connections_circle(i,2),:)== endpoints(Index_circle(j),:)
              circle_lane = [circle_lane;WP_from_app_LaneNum_new(Index_circle(j),7)]; 
         end
     end
 end
 %straight road
 for i = 1:size(connections_translation,1)
     Index_translation = find(ismember(startpoints, waypoints(connections_translation(i,1),:),'rows'));
     for j = 1:size(Index_translation,1)
         if waypoints(connections_translation(i,2),:)== endpoints(Index_translation(j),:)
              translation_lane = [translation_lane;WP_from_app_LaneNum_new(Index_translation(j),7)]; 
         end
     end
 end
 
 Route_id = size(connections_circle,1)+size(connections_translation,1); %route id
 Route_LaneNumber = [[1:Route_id]',[circle_lane;translation_lane]];
 
%% Clean variables
clearvars  translation_lane circle_lane endpoints i j Index_circle Index_translation Route_id startpoints WP_from_app WP_from_app_LaneNum WP_from_app_LaneNum_new waypoints_new a h  A alpha b B1 B2 C Circle D data E End F1 F2 F3 G1 G2 G3  m n rho1 rho2 rho3 Start tag theta1 theta2 theta3 alpha1 alpha2 x0 y0 waypoints_origin B1 k r R curvature 


function Waypoints_new_rightorder = Order_sequence(waypoints_ori,Waypoints_new)
% Compare the new waypoints with original waypoints to get right sequence
 Waypoints_new_rightorder = [];
 Waypoints_extra = [];
  g = 1;
 for k = 1: size(Waypoints_new,1)
     for t = 1: size(waypoints_ori,1)
         if Waypoints_new(k,1:3)== waypoints_ori(t,:) % the point is the same with one of the original point
            Waypoints_new_rightorder(t,:) = Waypoints_new(k,:);
            break; 
      
          elseif ismember(waypoints_ori,Waypoints_new(k,1:3),'rows')==0 % new added point for the map extension
              Waypoints_extra(g,:) = Waypoints_new(k,:);
              g = g+1;
              break;
         end
     end
 end
 Waypoints_new_rightorder = [Waypoints_new_rightorder;Waypoints_extra];

end