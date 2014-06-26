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

classdef EKFClass < handle
    %Class that applies the EKFs
    
    properties
        Q;
        P;
        R;
        G;
    end
    
    methods
        function ekf = EKFClass(aNoiseManager)
              
            % Define system noise covariance matrix Q
            Q11 = aNoiseManager.mSystemNoise ^ 2;
            Q22 = aNoiseManager.mSystemNoise ^ 2;
            Q33 = aNoiseManager.mAngleNoise ^ 2; 
            ekf.Q = diag([Q11,Q22,Q33]);

            % Error covariance matrix P
            ekf.P = ekf.Q; % initial value for P

            %Define measurement error covariance matrix R
            R11 = aNoiseManager.mScannerMeasureNoise ^ 2; %distance error = 0.01m
            R22 = aNoiseManager.mScannerAngleNoise ^ 2;   %bearing error = 0.048rad
            ekf.R = diag([R11,R22]);

            %Define input noise covariance matrix G
            G11 = aNoiseManager.mLeftWheelNoise ^ 2; %0.01m
            G22 = aNoiseManager.mRightWheelNoise ^ 2;
            ekf.G = diag([G11,G22]);          
        end
        
        function Draw(EKF, aAxes, aEstimatedPositionAndAngle)
            hold(aAxes, 'on');
            diameter = trace(EKF.P);
                     
            ang=0:0.01:2*pi; 
            xp=diameter/200*cos(ang);
            yp=diameter/200*sin(ang);
            
            %plot(aAxes, aEstimatedPosition(1)+xp, aEstimatedPosition(2)+yp, 'g');
            
            length = diameter/200;
            
            plot(aAxes, [aEstimatedPositionAndAngle(1) + length * cos(aEstimatedPositionAndAngle(3) + pi/2); aEstimatedPositionAndAngle(1) + length * cos(aEstimatedPositionAndAngle(3) - pi/2)], [aEstimatedPositionAndAngle(2) + length * sin(aEstimatedPositionAndAngle(3) + pi/2); aEstimatedPositionAndAngle(2) + length * sin(aEstimatedPositionAndAngle(3) - pi/2)], 'g');
            
            hold(aAxes, 'off');
        end
        
        function CalculateCovariance(EKF, RB, aDistanceAndAngle)
            
            % Create system matrix A
            dD = aDistanceAndAngle(1);
            %display('dD'); disp(dD);
            b=RB.mWheelDistance;
            %display('b'); disp(b);
            
            phiR = RB.GetEstimatedAngle();
            PhiM = (phiR - aDistanceAndAngle(2)) + aDistanceAndAngle(2)/2;
            
            A = eye(3,3);
            A(1,3) = -dD*sin(PhiM);
            A(2,3) = dD*cos(PhiM);
            %display('A'); disp(A);

            % Create input matrix B
            B(1,1) = 0.5*cos(PhiM) + dD/2/b*sin(PhiM);
            B(1,2) = 0.5*cos(PhiM) - dD/2/b*sin(PhiM);
            B(2,1) = 0.5*sin(PhiM) - dD/2/b*cos(PhiM);
            B(2,2) = 0.5*sin(PhiM) + dD/2/b*cos(PhiM);
            B(3,1) = -1/b;
            B(3,2) = 1/b;
            %display('B'); disp(B);

            % Create P
            EKF.P =A*EKF.P*A'+ B*EKF.G*B' + EKF.Q;
            %display('EKF.P'); disp(EKF.P);
            
            
        end
        
        
        function CorrectedPositionAndAngle = ApplyFilter(EKF, RB, landMarkFoundCell, aDistanceAndAngle)

            xR = RB.GetEstimatedX();
            yR = RB.GetEstimatedY();
            phiR = RB.GetEstimatedAngle();
          
            %for each landmark we can see
            for i = 1 : length(landMarkFoundCell)
                %get the first landMark: [x, y, distance, angle]
                xLM = landMarkFoundCell{i}(1);
                yLM = landMarkFoundCell{i}(2);
                distanceRobotLandMarkInMeter = landMarkFoundCell{i}(3);
                angleRobotToLandMarkInRad = landMarkFoundCell{i}(4);
                
                %display('landmarkData'); disp(landMarkFoundCell{i});              
                
                % Build measurement matrix H
                distanceSq = (xLM-xR)^2+(yLM-yR)^2;
                H(1,1) = (xR-xLM)/sqrt(distanceSq);
                H(1,2) = (yR-yLM)/sqrt(distanceSq);
                H(1,3) = 0;
                H(2,1) = (yLM-yR)/distanceSq;
                H(2,2) = (xR-xLM)/distanceSq;
                H(2,3) = -1;

                % Predicted measurements
                PredMeasure(1,1) = sqrt((xLM-xR)^2+(yLM-yR)^2); %d 
                PredMeasure(2,1) = atan2((yLM-yR),(xLM-xR))-phiR; %alpha
                % display('predicted measurement hehe');disp(PredMeasure());
                
                % Get the real noisy measurements
                RealMeasure(1,1) = distanceRobotLandMarkInMeter; %distance of landmark
                RealMeasure(2,1) = angleRobotToLandMarkInRad; %angle of landmark to robot's front vector

                % Calculate Kalman Gain
                K = EKF.P*H'*inv(H*EKF.P*H'+ EKF.R);

     
                % Update the state with measurements
                if PredMeasure(2,1)>pi
                    PredMeasure(2,1) = PredMeasure(2,1) -2*pi;              
                end

                ErrorMeasure = RealMeasure - PredMeasure;

                if ErrorMeasure(2,1)>pi
                    ErrorMeasure(2,1) = ErrorMeasure(2,1) - 2*pi;              
                end

                errorArray = K*ErrorMeasure;
                %display('errorArray'); disp(errorArray);
                               
                EKF.P = (eye(size(K,1))-K*H)*EKF.P;
                
                xR = xR + errorArray(1);
                yR = yR + errorArray(2);
                phiR = phiR + errorArray(3);
                
            end
        
            CorrectedPositionAndAngle(1) = xR;  % Update state using Kalman gain
            CorrectedPositionAndAngle(2) = yR;
            CorrectedPositionAndAngle(3) = phiR;
            %display('CorrectedPositionAndAngle'); disp(CorrectedPositionAndAngle);
        end
    end
    
end

