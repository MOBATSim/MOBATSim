function Waypoints_new_rightorder = Order_sequence(waypoints_ori,Waypoints_new)
% Compare the new waypoints with original waypoints to get right sequence
 Waypoints_new_rightorder = [];
 Waypoints_extra = [];
  g = 1;
 for k = 1: size(Waypoints_new,1)
     for t = 1: size(waypoints_ori,1)
         if Waypoints_new(k,:)== waypoints_ori(t,:) % the point is the same with one of the original point
            Waypoints_new_rightorder(t,:) = Waypoints_new(k,:);
            break; 
      
          elseif ismember(waypoints_ori,Waypoints_new(k,:),'rows')==0 % new added point for the map extension
              Waypoints_extra(g,:) = Waypoints_new(k,:);
              g = g+1;
              break;
         end
     end
 end
 Waypoints_new_rightorder = [Waypoints_new_rightorder;Waypoints_extra];