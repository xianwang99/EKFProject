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

classdef Robot < handle
 
    properties (SetAccess = private)
        %this is the actual pose which is: [x, y, angle in rad]
        mActualPositionAndAngle;
        
        %this is the estimated pose which is: [x, y, angle in rad]
        mEstimatedPositionAndAngle;
        
        %this is the b in meter
        mWheelDistance = 0.32;
        
        %max distance the scanner can scan in meters
        mScanLimitMeter;
        
        %min and max angle the scanner can can in radians
        mScanMinLimitRad;
        mScanMaxLimitRad;
        
        %noise manager
        mNM;
        
    end
    
    methods
        %aInitialPosition = [x,y,angle]
        function RB = Robot(aInitialPosition, aNoiseManager)
            RB.mActualPositionAndAngle = aInitialPosition;

            RB.mEstimatedPositionAndAngle = aInitialPosition;
                 
            RB.mScanLimitMeter = 10;
            RB.mScanMinLimitRad = -pi/2;
            RB.mScanMaxLimitRad = pi/2;
            
            RB.mNM = aNoiseManager;
        end
        
        function SetActualPositionAndAngle(RB, aActualPosition)
            RB.mActualPositionAndAngle = aActualPosition;
        end
        
        function u = GetActualPositionAndAngle(RB)
            u = RB.mActualPositionAndAngle;
        end
        
        function SetEstimatedPositionAndAngle(RB, aEstimatedPosition)
            RB.mEstimatedPositionAndAngle = aEstimatedPosition;
        end
        
        function u = GetEstimatedPositionAndAngle(RB)
            u = RB.mEstimatedPositionAndAngle;
        end 
        
        function angleRad = GetEstimatedAngle(RB)
            angleRad = RB.mEstimatedPositionAndAngle(3);
        end
        
        function SetEstimatedAngle(RB, aAngleRad)
            RB.mEstimatedPositionAndAngle(3)=aAngleRad;
        end
        
        function x = GetEstimatedX(RB)
            x = RB.mEstimatedPositionAndAngle(1);
        end

        function SetEstimatedX(RB, aX)
            RB.mEstimatedPositionAndAngle(1)=aX;
        end
        
        function y = GetEstimatedY(RB)
            y = RB.mEstimatedPositionAndAngle(2);
        end
        
        function SetEstimatedY(RB, aY)
            RB.mEstimatedPositionAndAngle(2)=aY;
        end
        
        
        function angleRad = GetActualAngle(RB)
            angleRad = RB.mActualPositionAndAngle(3);
        end
        
        function SetActualAngle(RB, aAngleRad)
            RB.mActualPositionAndAngle(3)=aAngleRad;
        end
        
        function x = GetActualX(RB)
            x = RB.mActualPositionAndAngle(1);
        end

        function SetActualX(RB, aX)
            RB.mActualPositionAndAngle(1)=aX;
        end
        
        function y = GetActualY(RB)
            y = RB.mActualPositionAndAngle(2);
        end
        
        function SetActualY(RB, aY)
            RB.mActualPositionAndAngle(2)=aY;
        end
               
        function Turn(RB, aAngleInRad)
            if mod(aAngleInRad, 2*pi) == 0
                %no angle no need to turn;
                return
            end
            
            %update the estimated angle, which is simply adding the 
            %the angle we want
            RB.SetEstimatedAngle((RB.GetEstimatedAngle() + aAngleInRad));
            
            
            %update the actual angle which has noise
            angleNoise = aAngleInRad + RB.mNM.GenerateAngleNoise();            
            RB.SetActualAngle( RB.GetActualAngle() + angleNoise);
        end
        
        function Move(RB, aDistanceInMeter)
            %update the estimated position
            RB.SetEstimatedX(RB.GetEstimatedX() + aDistanceInMeter * cos(RB.GetEstimatedAngle()));
            RB.SetEstimatedY(RB.GetEstimatedY() + aDistanceInMeter * sin(RB.GetEstimatedAngle()));
                
            %update the actual position
            distanceLeftWheel = aDistanceInMeter + RB.mNM.GenerateLeftWheelNoise(); 
            distanceRightWheel = aDistanceInMeter + RB.mNM.GenerateRightWheelNoise();
            
            distanceRobot = (distanceLeftWheel + distanceRightWheel) / 2;
            RB.SetActualX( RB.GetActualX() + distanceRobot * cos(RB.GetActualAngle()));
            RB.SetActualY(RB.GetActualY() + distanceRobot * sin(RB.GetActualAngle()));
       end
        
        function DrawRobot(RB, aAxes)
            
             hold(aAxes, 'on');
             ang=0:0.01:2*pi; 
             xp=RB.mWheelDistance/2*cos(ang);
             yp=RB.mWheelDistance/2*sin(ang);
             
             angScanner = RB.GetActualAngle() + RB.mScanMinLimitRad :0.01:RB.GetActualAngle() + RB.mScanMaxLimitRad;
             angX = RB.mScanLimitMeter * cos(angScanner);
             angY = RB.mScanLimitMeter * sin(angScanner);
             
             %draw estimated robot
             plot(aAxes, RB.GetEstimatedX()+xp, RB.GetEstimatedY()+yp, 'k'); 
             plot(aAxes, [RB.GetEstimatedX() RB.GetEstimatedX()+cos(RB.GetEstimatedAngle())],[RB.GetEstimatedY()  RB.GetEstimatedY()+sin(RB.GetEstimatedAngle())],'k')
             
             %draw actual robot
             plot(aAxes, RB.GetActualX()+xp, RB.GetActualY()+yp, 'r');
             plot(aAxes, [RB.GetActualX() RB.GetActualX()+cos(RB.GetActualAngle())],[RB.GetActualY()  RB.GetActualY()+sin(RB.GetActualAngle())],'r')
         
             %draw the scanner's limit
             plot(aAxes, RB.GetActualX()+angX, RB.GetActualY()+angY, 'y');
             plot(aAxes, [RB.GetActualX() + RB.mScanLimitMeter * cos(RB.GetActualAngle() + pi/2); RB.GetActualX() + RB.mScanLimitMeter * cos(RB.GetActualAngle() - pi/2)], [RB.GetActualY() + RB.mScanLimitMeter * sin(RB.GetActualAngle() + pi/2); RB.GetActualY() + RB.mScanLimitMeter * sin(RB.GetActualAngle() - pi/2)], 'y');
             hold (aAxes, 'off');
        end 
        
        function DrawTrace(RB, aAxes)
            
             hold(aAxes, 'on');

             %draw estimated robot
             plot(aAxes, RB.GetEstimatedX(), RB.GetEstimatedY(), 'k');
             %plot(aAxes, [RB.GetEstimatedX() RB.GetEstimatedX()+cos(RB.GetEstimatedAngle())],[RB.GetEstimatedY()  RB.GetEstimatedY()+sin(RB.GetEstimatedAngle())],'k')
             
             %draw actual robot
             plot(aAxes, RB.GetActualX(), RB.GetActualY(), 'r');
             %plot(aAxes, [RB.GetActualX() RB.GetActualX()+cos(RB.GetActualAngle())],[RB.GetActualY()  RB.GetActualY()+sin(RB.GetActualAngle())],'r')
         
             hold (aAxes, 'off');
        end
        
        %returns [x, y, distance, angle]
        function LandMarksFoundCell = ScanAndDraw(RB, aLandMarkManager, aAxes)
            nbFound = 0;
            %LandMarksFoundCell = cell(aLandMarkManager.mNbLandMarks);
            for i = 1 : aLandMarkManager.mNbLandMarks
                
                Vector1 = [RB.GetActualX(), RB.GetActualY()] - [RB.GetActualX()+cos(RB.GetActualAngle()), RB.GetActualY()+sin(RB.GetActualAngle())];
                Vector2 = [RB.GetActualX(), RB.GetActualY()] - [aLandMarkManager.mLandMarksCell{i}(1), aLandMarkManager.mLandMarksCell{i}(2)];
                angle = atan2( det([Vector1;Vector2;]) , dot(Vector1,Vector2) );

                distance = (((RB.GetActualX() - aLandMarkManager.mLandMarksCell{i}(1))^2 + (RB.GetActualY()-aLandMarkManager.mLandMarksCell{i}(2))^2)^0.5);
                if ( distance <= RB.mScanLimitMeter && ( angle >= RB.mScanMinLimitRad && angle <= RB.mScanMaxLimitRad ) )
                    nbFound = nbFound + 1;
                    
                    distanceAndNoise = distance + RB.mNM.GenerateScannerMeasureNoise();
                    if(distanceAndNoise < 0)
                        distanceAndNoise = 0;
                    end
                    angleAndNoise = angle + RB.mNM.GenerateScannerAngleNoise();
                    
                    %display('landmark measurement with/without noise')
                    %disp([distance, distanceAndNoise, angle, angleAndNoise]);
                    
                    LandMarksFoundCell{nbFound} = [aLandMarkManager.mLandMarksCell{i}(1), aLandMarkManager.mLandMarksCell{i}(2), distanceAndNoise, angleAndNoise];
                    
                    %draw a line form our robot to the landmark it scanned
                    %line([RB.GetActualX(), LandMarksFoundCell{nbFound}(1)], [RB.GetActualY(), LandMarksFoundCell{nbFound}(2)]); 
                    
                    %draw the noisy detection
                    hold (aAxes, 'on');
                    plot(aAxes, [RB.GetActualX(); RB.GetActualX() + distanceAndNoise * cos(angleAndNoise+RB.GetActualAngle())], [RB.GetActualY(); RB.GetActualY() + distanceAndNoise * sin(angleAndNoise+RB.GetActualAngle())]); 
                    hold (aAxes, 'off');
                end
            end
            if nbFound == 0
                LandMarksFoundCell = [];
            end
        end
    end
end

