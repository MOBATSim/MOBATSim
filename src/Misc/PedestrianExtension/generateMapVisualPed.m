function generateMapVisualPed(obj,displayInGridCoordinates)
            %This function plots any XML Map of MOBATSim. Keep in mind that you have to
            %do a coordinate transformation between normal coordinates and grid / mobatsim
            %Input: XML Map object of MOBATSim, boolean wether to plot mobatsim or grid coordinates
            %Output TODO return the plot to use it later

            %% prepare everything
            %open another figure
            %figure(3);
            %plot(1,1,'color',[1,1,1]);%TODO replace with better solution for deleting old plot
            hold on            
            w = obj.waypoints;
            circ = obj.connections.circle;
            trans = obj.connections.translation;
            %coordinate transformation
            w(:,3) = -1.*w(:,3);
            circ(:,6) = -1.*circ(:,6);
            if displayInGridCoordinates
                xOff = min(w(:,1))-50;  %TODO make it global later
                yOff = min(w(:,3))-50;  %offset should be global
                
                w(:,3) = w(:,3)-yOff;
                w(:,1) = w(:,1)-xOff;                
                
                circ(:,4) = circ(:,4)-xOff;
                circ(:,6) = circ(:,6)-yOff;
            end
            %% Generate a usable plot
            %% generate curves
            for c = 1 : length(circ)
                cPart = circ(c,:);
                %start
                x1 = w(cPart(1),1); 
                y1 = w(cPart(1),3);   
                %goal
                x2 = w(cPart(2),1); 
                y2 = w(cPart(2),3);
                %central point
                x0W = cPart(4); 
                y0W = cPart(6);
                %radius
                radius = norm( [x2,y2]-[x0W,y0W] );
                %angles
                phiStart = angle(complex((x1-x0W) , (y1-y0W)));
                phiGoal = angle(complex((x2-x0W) , (y2-y0W)));
                %direction
                direction = sign(cPart(3));
                %% make angle allways usable
                if phiStart <0
                    phiStart = phiStart + 2*3.1415;
                end
                if phiGoal <0
                    phiGoal = phiGoal + 2*3.1415;
                end
                %make turns through 0° possible
                if (direction == -1 && phiStart < phiGoal)
                    phiStart = phiStart + 2*3.1415;
                end
                if (direction == 1 && phiStart > phiGoal)
                    phiGoal = phiGoal + 2*3.1415;
                end
                phi1 = phiStart : direction*0.01 : phiGoal;
                phi1(1) = phiStart;
                phi1(end) = phiGoal;
                points = [(radius .* cos(phi1)+x0W)',(radius .* sin(phi1))'+y0W];
                
                plot(points(:,1),points(:,2),'color',[0 1 0],'LineWidth',4);
                %plot number next to edge
%                 textPos = points(round(length(points)/2,0),:);                
%                 description = text(textPos(1)-10,textPos(2)-15,num2str(c),'color',[0 0.5 0]);
            end
            %% generate straight lines
            for t = 1 : length(trans)
                position = zeros(2,2);
                %get both points and plot a line in between
                position(1,:) = [w(trans(t,1),1) ,w(trans(t,1),3)];
                position(2,:) = [w(trans(t,2),1) ,w(trans(t,2),3)];
                plot(position(:,1),position(:,2),'color',[0 1 0],'LineWidth',4);
                %plot number next to edge
%                 textPos = (position(2,:) + position(1,:))/2;                
%                 text(textPos(1)+5,textPos(2)-15,num2str(t+c),'color',[0 0.5 0]);
            end
            %% plot nodes with numbers
            for n = 1 : length(w)
                pos = [w(n,1),w(n,3)];
                plot(pos(1),pos(2),'Marker','o','MarkerFaceColor',[0 1 0],'color',[0 1 0],'MarkerSize',3);
%                 text(pos(1)-5,pos(2)-15,num2str(n),'color',[0 0.2 0.5]);%TODO make it appear automatically under dot
                %maybe there is a way to let it not collide with other text?
            end
            hold off
        end