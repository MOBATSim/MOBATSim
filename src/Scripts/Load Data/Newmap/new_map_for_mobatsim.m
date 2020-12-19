%clear all
%clc

load new_map.mat; %Load file from DrivingScenarioDesigner
load waypoints_origin.mat;

%% Initialize
Waypoints_new = [];
connections_translation = [];
connections_circle = [];
waypoints = [];
%% Get waypoints
% Get all the waypoints from drivingScenarioDesigner output file
Start = data.RoadSpecifications(1,1).Centers(1,:);
End = data.RoadSpecifications(1,1).Centers(end,:);
A = [Start;End];

for i = 2:94
    %A = [A; data.RoadSpecifications(1,i).Centers(1,:);data.RoadSpecifications(1,i).Centers(end,:)];
    A = [A; data.RoadSpecifications(1,i).Centers];
end
B1 = unique(A,'row','stable');

%coordinate transformation(from drivingScenariodesigner coordimat to MOBatsim coordimate)
% [y,-x]->[x,-y]
Waypoints_new(:,1) = -B1(:,2);
Waypoints_new(:,3) = -B1(:,1);
%put the new waypoints in the same sequence as before
waypoints =  Order_sequence(waypoints_origin,Waypoints_new);
B2 = [];
B2(:,1) = -waypoints(:,3);
B2(:,2) = -waypoints(:,1);
B2(:,3) = 0;
%% Get connection_transition
for j = 1:55
    a = ismember(B2, data.RoadSpecifications(1,j).Centers(1,:),'rows');
    m = find(a==1);
    b = ismember(B2, data.RoadSpecifications(1,j).Centers(2,:),'rows');
    n = find(b==1);
    connections_translation(j,:) = [n m 100];% find the sequence of translation connection
   
end
%% Get connection_circle
Circle = zeros(1,7);
for j = 56:94
    % find three points on tne arc
    C = data.RoadSpecifications(1,j).Centers(1,1:2);
    D = data.RoadSpecifications(1,j).Centers(2,1:2);
    E = data.RoadSpecifications(1,j).Centers(end,1:2);
    % calculate the circle center 
    [x0,y0] = Circle_center(C,D,E);
 
    F1 = data.RoadSpecifications(1,j).Centers(end,:);
    F2 = data.RoadSpecifications(1,j).Centers(1,:);
    
    a = ismember(B2, F1,'rows'); % find the sequence of circle connection
    m = find(a==1);
    b = ismember(B2, F2,'rows');
    n = find(b==1);
        
    %polar coordinate
    F1 = F1(1:2);
    F2 = F2(1:2);
    G1 = F1-[x0,y0];
    G2 = F2-[x0,y0];
    [theta1, rho1] = cart2pol(G1(1),G1(2));
    [theta2, rho2] = cart2pol(G2(1),G2(2));
    if theta1 < 0
        theta1 = theta1+2*pi;
    end
    if theta2 < 0
        theta2 = theta2+2*pi;
    end
    %disp([j-55, 0, theta1, theta2]);
    alpha = theta2-theta1; % caltulate the the angle of arc
    if alpha > pi % if arc accross the positive part of x axis
        alpha = alpha - 2*pi;
    elseif alpha < -pi
        alpha = alpha + 2*pi;
    elseif abs(alpha) == pi
        F3 = data.RoadSpecifications(1,j).Centers(2,:);
        F3 = F3(1:2);
        G3 = F3-[x0, y0];
        [theta3, rho3] = cart2pol(G3(1),G3(2));
        alpha1 = theta3-theta1;
        alpha2 = theta2-theta3;
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
        alpha = alpha1+alpha2; 
    end
    Circle(1,1) = m;
    Circle(1,2) = n;
    Circle(1,3) = alpha;
    Circle(1,4) = -y0;
    Circle(1,5) = 0;
    Circle(1,6) = -x0;
    Circle(1,7) = 100;
    
    connections_circle = [connections_circle; Circle];   
end    
%% Clean variables
clearvars a A alpha b B1 B2 C Circle D data E End F1 F2 F3 G1 G2 G3 i j m n rho1 rho2 rho3 Start tag theta1 theta2 theta3 alpha1 alpha2 x0 y0 waypoints_origin Waypoints_new



