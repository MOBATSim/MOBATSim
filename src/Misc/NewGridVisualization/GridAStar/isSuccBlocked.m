function blocked = isSuccBlocked(succKey,futureData,arrivalTime)
            %this function returns true, if the node is blocked
            %FD [carID, coordinates of GL x, y, speed, time, deviation]
            coordinates = str2num(succKey);
            %get every entry in fd that has the same coordinates and
            %negative sigma (is blocked)
            curFD = futureData(futureData(:,2) == coordinates(1) & futureData(:,3) == coordinates(2) & futureData(:,6) < 0,:);            
            %now check, if someone would block it before we arrive
            for j = 1 : size(curFD,1)
%                 x = [0,arrivalTime];
%                 mu = curFD(j,5);
%                 sigma = curFD(j,6);
%                 pbb = normcdf(x,mu,sigma);
%                 p = pbb(2)-pbb(1);
%                 if p > 0
%                     blocked = true;
%                     return;
%                 end
                    if arrivalTime > curFD(j,5)
                        blocked = true;
                        return;
                    end
            end
            blocked = false;
        end