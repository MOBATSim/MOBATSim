function generate_rotation_WPs(car, speed, rotation_point,rotation_angle,Destination)

car.dynamics.localWPlist = [car.dynamics.position(1) -car.dynamics.position(3) car.dynamics.orientation(4)];

while 1
    
r = norm(Destination-rotation_point);

if car.pathInfo.routeCompleted == true
    
    car.dynamics.cornering.angles = pi;
    car.pathInfo.routeCompleted = false;
    
    point_to_rotate= car.dynamics.position;
    
    car.dynamics.cornering.a=point_to_rotate(1)-rotation_point(1);
    car.dynamics.cornering.b=point_to_rotate(2)-rotation_point(2);
    car.dynamics.cornering.c=point_to_rotate(3)-rotation_point(3);
    
end

vector_z=[0 0 1];

a = car.dynamics.cornering.a;
b = car.dynamics.cornering.b;
c = car.dynamics.cornering.c;

step_length = speed/r;
car.dynamics.cornering.angles = car.dynamics.cornering.angles - step_length;
t = car.dynamics.cornering.angles;

if  pi-rotation_angle>t
    car.dynamics.position = Destination;
    car.pathInfo.routeCompleted = true;
    car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
    
    %% TODO - check if it works in all situations
    idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
    if idx+1<=length(car.pathInfo.path)
        car.pathInfo.currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
    end
    %%
    car.dynamics.cornering.angles = 0;

    % Complete list of local waypoints to execute the turn left maneuver
    car.dynamics.localWPlist = [car.dynamics.localWPlist; car.dynamics.position(1) -car.dynamics.position(3) -theta];
    break;
end
vector_velocity=[-a*sin(t)-cos(t)*c b a*cos(t)-c*sin(t)];
vector=cross(vector_velocity, vector_z);
vector=vector/norm(vector);
theta=acos(dot(vector_velocity, vector_z)/(norm(vector_velocity)*norm(vector_z)));

car.dynamics.position = [rotation_point(1)-(a*cos(t)-sin(t)*c) rotation_point(2)+b*t rotation_point(3)-(a*sin(t)+c*cos(t))];
car.dynamics.orientation = [vector -theta];

car.dynamics.localWPlist = [car.dynamics.localWPlist; car.dynamics.position(1) -car.dynamics.position(3) -theta];

end
end