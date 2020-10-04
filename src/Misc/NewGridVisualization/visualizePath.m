function visualizePath(obj,displayStyle)
            %shows binary occupancy grid with the path of every vehicle
            %Input: boolean, should we plot the grid or the normal plot?
            %Only use true, if you have created the bog globaly!
            
            map = obj.vehicle.map;
            figure(4)%figure(2) causes errors
            
            if strcmp(displayStyle, "bog")
                show(obj.mapBOG)
                contrastArray = [1 1 1];
                displayInGridCoordinates = true;
            elseif strcmp(displayStyle, "bogPlot")  
                displayInGridCoordinates = true;
                contrastArray = [0 0 0];
                obj.generateMapVisual(map,true);%TODO use only the plot later 
            else
                displayInGridCoordinates = false;
                contrastArray = [0 0 0];
                obj.generateMapVisual(map,false);%TODO use only the plot later
            end
            hold on
            obj.plotPath(map,displayInGridCoordinates,contrastArray);
            hold off
        end