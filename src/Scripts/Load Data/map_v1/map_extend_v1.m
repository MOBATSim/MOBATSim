function [Route_LaneNumber, waypoints, connections_translation, connections_circle] = map_extend_v1()
    %% Load data file
    load('map_v1.mat','data');  % Map data (.mat format) taken as the OUTPUT of the "drivingScenarioDesigner App"

    % Original map waypoints to put the converted waypoints from OPENDrive into the right order of sequence in MOBATSim
    load('waypoints_origin.mat','waypoints_origin');  % (Important to keep the crossroad waypoints correct)

    %% Initialization
    waypoints_new = [];% waypoints of the extended map before the right sequence is ordered
    WP_from_app = []; % waypoints got from drivingScenarioDesigner

    %% waypoints
    % Get all the waypoints from drivingScenarioDesigner output file
    % Put all the STARTING and ENDING points of the CENTER LANES together
    for i = 1:size(data.RoadSpecifications,2)
        WP_from_app = [WP_from_app;
            data.RoadSpecifications(1,i).Centers(1,:);
            data.RoadSpecifications(1,i).Centers(end,:)];
    end

    % Remove all the repeated WAYPOINTS with the same coordinates in drivingScenarioDesigner
    uniqueWaypoints = unique(WP_from_app(:,1:3),'row','stable'); % This is necessary because more than one ROUTE might have the same merging WAYPOINT

    %% Coordinate transformation
    % Coordinate transformation(drivingScenarioDesigner APP ->MOBATSim coordinate system)
    % [y,-x,0]->[x,0,-y] :
    % Take the second column (APP), multiply by -1, make it the first column (MOBATSim)
    % Take the first column (APP), multiply by -1, make it the third column (MOBATSim)
    waypoints_new(:,1) = -uniqueWaypoints(:,2);
    waypoints_new(:,3) = -uniqueWaypoints(:,1);
    % Put the new waypoints into order to have the right sequence as previous MOBATKent Map -> (Important to keep the crossroad waypoints correct)
    waypoints =  Order_sequence(waypoints_origin,waypoints_new);

    %% Conversion back to DrivingScenarioDesigner APP
    % after sequence order, transfer MOBATSim coordinate to drivingScenariodesigner coordinate
    MOBATSim_WPs = [-waypoints(:,3) -waypoints(:,1) zeros(length(waypoints),1)];

    [connections_translation,connections_circle] = convertBackToMOBATSimfromAPP(MOBATSim_WPs,data);

    %% Fine lane number of corresponding route_id
    Route_LaneNumber =  getLaneNumber(data,waypoints,connections_circle,connections_translation);

end

function Waypoints_new_rightorder = Order_sequence(waypoints_ori,Waypoints_new)
    % Compare the new WAYPOINTS with original WAYPOINTS to get the right
    % sequence, add the new WAYPOINTS after the original WAYPOINTS
    idx = ~ismember(Waypoints_new,waypoints_ori,'rows');

    Waypoints_new = Waypoints_new(idx,:,:);

    Waypoints_new_rightorder = [waypoints_ori; Waypoints_new];

end


function [connections_translation,connections_circle] = convertBackToMOBATSimfromAPP(WPs,data)
connections_circle = [];
connections_translation = [];

%% Loop through all RoadSpecifications
for j = 1:size(data.RoadSpecifications,2)
    %% connection_transition
    if size(data.RoadSpecifications(1,j).Centers,1)==2 % if there are only 2 points in one road section, it is straight road
        a = ismember(WPs, data.RoadSpecifications(1,j).Centers(1,:),'rows');
        m = find(a==1); %index of end point of transition
        b = ismember(WPs, data.RoadSpecifications(1,j).Centers(2,:),'rows');
        n = find(b==1); % index of start point of transition
        connections_translation = [connections_translation; n m 100];% find the sequence of translation connections
        %% connection_circle
    elseif size(data.RoadSpecifications(1,j).Centers,1)>2 % if there are more than 2 points in one road section, it is arc
        % Find three points on tne arc
        WP_end = data.RoadSpecifications(1,j).Centers(end,:); % end point of the curved road
        WP_middle =  data.RoadSpecifications(1,j).Centers(2,:); % one point between start and end point
        WP_start = data.RoadSpecifications(1,j).Centers(1,:);  % start point of the road
        
        % Calculate the circle center
        [x0, y0, r] = Circle_center(WP_start(1:2),WP_middle(1:2),WP_end(1:2));
        k = 1/r; % calculate curvature for the future speed limit

        a = ismember(WPs, WP_end,'rows'); % find the sequence of circle connection
        m = find(a==1);
        b = ismember(WPs, WP_start,'rows');
        n = find(b==1);
        
        % Polar coordinate
%         WP_end_xy = WP_end(1:2);
%         WP_start_xy = WP_start(1:2);
        WP_end_rela = WP_end(1:2)-[x0,y0];% the coordinate of WP_end relative to circle center
        WP_start_rela = WP_start(1:2)-[x0,y0];
        [theta1, rho1] = cart2pol(WP_end_rela(1),WP_end_rela(2));% cartesian coordinate to polar coordinate of G1
        [theta2, rho2] = cart2pol(WP_start_rela(1),WP_start_rela(2));
        if theta1 < 0 % [-pi~pi](MATLAB range)->[0~2pi](MOBATSim range) 
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
            WP_middle_rela = WP_middle(1:2)-[x0, y0];
            [theta3, rho3] = cart2pol(WP_middle_rela(1),WP_middle_rela(2));
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

        connections_circle = [connections_circle; m n alpha -y0 0 -x0 100];
    end
end
end

function [x0, y0, r] = Circle_center(A,B,C)% this function use three points on the arc to calculate the circle center
% just use three points on the arc
% A and B are start and end points
% C is the middle point of the arc
syms x y r % three unknown parameters: the coordinate of the circle center and the radius
eqn1 = r==(x-A(1))^2+(y-A(2))^2; %according to the stantard equation of the circle :(x-a)^2+(y-b)^2 = r^2
eqn2 = r==(x-B(1))^2+(y-B(2))^2;
eqn3 = r==(x-C(1))^2+(y-C(2))^2;

[x,y,r]=solve(eqn1, eqn2, eqn3, x, y, r);% solve three equation to get three unknown parameters 
x = double(x);
y = double(y);
r = double(r);
r = sqrt(r);

x0 = roundn(x, -2);% rounded to 2 digits
y0 = roundn(y, -2);
r  = roundn(r,-2); 
end

%% start to find lane number for every connections
function Route_LaneNumber = getLaneNumber(data,waypoints,connections_circle,connections_translation)
WP_from_app_LaneNum = [];%[start, end, laneNumber]
%get the waypoints and the coorsponding lane number from map data
for i = 1:size(data.RoadSpecifications,2)
    WP_from_app_LaneNum = [WP_from_app_LaneNum; 
                  data.RoadSpecifications(1,i).Centers(1,:),data.RoadSpecifications(1,i).Centers(end,:),data.RoadSpecifications(1,i).Lanes.NumLanes];   
end

%%coordinate transformation
% [y,-x,0]->[x,0,-y]
%[end_x,0,-end_y,start_x,0,-start_y,lane_number]
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
end 