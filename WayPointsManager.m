%Copyright (c) 2014 Xian Wang.
%All rights reserved.

%Redistribution and use in source and binary forms are permitted
%provided that the above copyright notice and this paragraph are
%duplicated in all such forms and that any documentation,
%advertising materials, and other materials related to such
%distribution and use acknowledge that the software was developed
%by Xian Wang. The name of
%Xian Wang may not be used to endorse or promote products derived
%from this software without specific prior written permission.
%THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
%IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
%WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

classdef WayPointsManager < handle

    properties
        mWayPointsCell;
        mNbWayPoints=0;
        
        %first waypoint is the initial position, hence it's reached
        mWayPointReached=1;
        
        mMaxDistanceInMeterPerStep;
    end
    
    methods
        function WPM = WayPointsManager(aMaxDistanceInMeterPerStep) 
            WPM.mMaxDistanceInMeterPerStep = aMaxDistanceInMeterPerStep;
        end
        
        function AddWayPoint(WPM, aPosition)
            WPM.mNbWayPoints = WPM.mNbWayPoints + 1;
            WPM.mWayPointsCell{WPM.mNbWayPoints} = aPosition;
        end
        function nbWayPoints = GetNbWayPoints(WPM)
            nbWayPoints = WPM.mNbWayPoints;
        end
        function Draw(WPM, aAxes)
            hold on;
            for i = 1 : WPM.mNbWayPoints
                if (i == WPM.mWayPointReached + 1)
                    plot(aAxes, WPM.mWayPointsCell{i}(1), WPM.mWayPointsCell{i}(2), 'g+');
                else
                    plot(aAxes, WPM.mWayPointsCell{i}(1), WPM.mWayPointsCell{i}(2), 'k+');
                end
                if (i > 1)
                    plot(aAxes, [WPM.mWayPointsCell{i-1}(1),WPM.mWayPointsCell{i}(1)], [WPM.mWayPointsCell{i-1}(2),WPM.mWayPointsCell{i}(2)], 'b--');
                end
            end
            hold off;
        end
        
        function DistanceAndAngle = GetNextStep(WPM, aPositionAndAngle)
            %the goal is to figure out the angle and the distance to
            %robot should do to reach the next way point.
            
            %get the next waypoint
            Vector1 = [aPositionAndAngle(1), aPositionAndAngle(2)] - [aPositionAndAngle(1)+cos(aPositionAndAngle(3)), aPositionAndAngle(2)+sin(aPositionAndAngle(3))];
            Vector2 = [aPositionAndAngle(1), aPositionAndAngle(2)] - [WPM.mWayPointsCell{WPM.mWayPointReached+1}(1), WPM.mWayPointsCell{WPM.mWayPointReached+1}(2)];
            
            %get the angle between the Robot's angle and the waypoint
            
            angle = atan2( det([Vector1;Vector2;]) , dot(Vector1,Vector2) );
            
            %now get the distance
            
            distance = (((aPositionAndAngle(1) - WPM.mWayPointsCell{WPM.mWayPointReached+1}(1))^2 + (aPositionAndAngle(2)-WPM.mWayPointsCell{WPM.mWayPointReached+1}(2))^2)^0.5);

            if distance <= WPM.mMaxDistanceInMeterPerStep
                %we consider the robot will reach that waypoint in the next
                %move
                WPM.mWayPointReached = WPM.mWayPointReached + 1;
                
                DistanceAndAngle = [distance, angle];
                return
            end
            DistanceAndAngle = [WPM.mMaxDistanceInMeterPerStep, angle];
            %display('DistanceAndAngle');disp(DistanceAndAngle);
        end
    end
end

