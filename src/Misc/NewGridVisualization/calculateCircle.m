function pixelArray = calculateCircle(pStart,phiStart,pGoal,phiGoal,pCenter,direction,radius,offset)

    %start with the first one
    curPix = pStart;
    %assign it to an array
    pixelArray = [];%store points here
    pixelArray = [pixelArray,append(num2str(curPix(1)), ",", num2str(curPix(2)))];
    curPhi = phiStart;
    while curPix(1) ~= pGoal(1) || curPix(2) ~= pGoal(2)
        %while the goal pixel is not reached, go to the next
        %pixel, that is between the last one and the goal and
        %is closest to the radius
        nextPix = [];   %the next pixel to draw in grid
        nextPhi = 0;
        deltaR = 200000;     %set up distance to the radius point with angle phi to be allways higher first try
        %for every neighbour
        for x = -1 : 1
            for y = -1 : 1
                if x ~= 0 || y ~= 0
                    %compare pixel to get the closest one to the original
                    %point
                    %calculate the distance between the pixel and the
                    %reference and use the closest
                    neighbourPix = [curPix(1)+x,curPix(2)+y];%new pixel in grid
                    phi = angle(complex((neighbourPix(1)-pCenter(1)) , (neighbourPix(2)-pCenter(2))));%angle in world
                    if phi < 0
                        phi = phi + 2*3.1415;
                    else
                        phi = phi + offset;
                    end
                    %test, if the angle is relevant (between last and goal)
                    if (direction == 1 && curPhi <= phi && phiGoal >= phi)|| (direction ==-1 && curPhi >= phi && phiGoal <= phi)
                        %get point with same angle on the radius
                        referencePoint = [radius * cos(phi)+pCenter(1),radius*sin(phi)+pCenter(2)];
                        %calculate distance between reference and current neighbour pixel
                        refDeltaR = norm(neighbourPix - referencePoint);
                        if refDeltaR < deltaR
                            %if the distance is less then
                            %previously, we found a better next
                            %pixel
                            deltaR = refDeltaR;
                            nextPix = neighbourPix;
                            nextPhi = phi;
                        end
                    end
                end
            end
        end
        %move to the best pixel
        curPix = nextPix;
        curPhi = nextPhi;
        %now curPix is the next pixel to draw
        pixelArray = [pixelArray, append(num2str(curPix(1)), ",", num2str(curPix(2)))];
    end
end

