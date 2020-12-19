function Waypoints_new_rightorder = Order_sequence(waypoints,Waypoints_new)
% compare the waypoints from drivingScenarioDesigner to the original
% waypoints to order the sequence
 Waypoints_new_rightorder = [];
 Waypoints_extra = [];
 g = 1;
 for k = 1:size(Waypoints_new,1)
     for t = 1:size(waypoints,1)
         if Waypoints_new(k,:)==waypoints(t,:)
             Waypoints_new_rightorder(t,:) = Waypoints_new(k,:);  
%              break;
%          elseif any(waypoints==Waypoints_new(k,:))==[0 1 0]
%              Waypoints_extra(g,:) = Waypoints_new(k,:);
%              g = g+1;
%              break;
         end
     end
 end
 Waypoints_new_rightorder = [Waypoints_new_rightorder;Waypoints_extra];