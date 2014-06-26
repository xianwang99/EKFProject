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
%WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.s

classdef Simulation < handle
    
    properties
        %robot
        RB;
        %landmark manager
        LMM;
        %waypoint manager
        WPM;
        %EKF class
        EKF;  
        %NoiseManager
        NM;
        mSimulationFinished=0;
        mNbRobots=0;
        mStopSimulation=0
        mAxis=[-5,30,-5,35];
    end
    
    methods
        %constructor
        function sim = Simulation()
            sim.Reset(); 
        end
        function Reset(sim)
            clear Robot;
            clear LandMarksManager;
            clear WayPointsManager;
            clear EKFClass;
            clear NoiseManager;
            
            clear sim.RB;
            clear sim.WPM;
            clear sim.NM;
            clear sim.EKF;
            clear sim.LMM;
            sim.mSimulationFinished=0;
            sim.mNbRobots=0;
            sim.mStopSimulation=0;
            sim.mAxis=[-5,30,-5,35];
            
            sim.NM = NoiseManager();
            sim.LMM = LandMarksManager();

        end
        function StopSimulation(sim)
            sim.mStopSimulation = 1;
        end

        function SimulateRobot(sim, aHandles)
            
            if (sim.mNbRobots == 0)
                return;
            end
            
            
            %create the robot and set its initial position to the first 
            %waypoint
           
            if(sim.mStopSimulation == 0)
                cla(aHandles.axes2);
                axes(aHandles.axes2)
                axis(sim.mAxis);
            else
                sim.mStopSimulation = 0;
            end
            
            robotsFinished = 0;
            while (sim.mStopSimulation == 0)
                if(robotsFinished == sim.mNbRobots)
                    sim.mSimulationFinished = 1;
                    set(aHandles.StartEval, 'String', 'START Simulation');
                    set(aHandles.OpenDataFileBtn, 'Enable', 'on');
                    break;
                end
                robotsFinished = 0;
                for i = 1 : sim.mNbRobots
                    if (sim.WPM{i}.GetNbWayPoints() <= 1)
                        %nothing to do we reached the end...
                        continue;
                    end
                    %clear the graph
                    if (i == 1)
                        cla(aHandles.axes1);
                        axes(aHandles.axes1);
                        axis(sim.mAxis);
                        
                        %draw the landmarks
                        sim.LMM.Draw(aHandles.axes1);
                    end
                    %stop if we reached the end
                    if (sim.WPM{i}.mWayPointReached == sim.WPM{i}.mNbWayPoints)
                        sim.WPM{i}.Draw(aHandles.axes1);
                        sim.RB{i}.DrawRobot(aHandles.axes1);
                        robotsFinished = robotsFinished + 1;  
                        continue;
                    end
                    
                    sim.WPM{i}.Draw(aHandles.axes1);

                    %the order of things is pretty simple its:
                    %1) Turn the robot to the waypoint it wants to go
                    %2) Move the robot forward toward that waypoint
                    %3) Scan the area, 2 scenarios can happen, a) and b)

                    %4)a) no landmark found, continue the loop Finish goto 1)

                    %4)b) landmarks are found, get our position via EKF
                    %5)update estimated position with the EKF position
                    %6)goto 1)


                    %Ask the controller where to go next
                    DistanceAndAngle = sim.WPM{i}.GetNextStep(sim.RB{i}.GetEstimatedPositionAndAngle());

                    %turn by what the controller gave us
                    sim.RB{i}.Turn(DistanceAndAngle(2));

                    %move by what the controller gave us
                    sim.RB{i}.Move(DistanceAndAngle(1));

                    %display('actual position and angle'); disp(sim.RB.GetActualPositionAndAngle());
                    %display('estimate position and angle'); disp(sim.RB.GetEstimatedPositionAndAngle());

                    %now try to draw line from scanner to landmarks
                    landMarkFoundCell = sim.RB{i}.ScanAndDraw(sim.LMM, aHandles.axes1);

                    %draw robot before the correction
                    sim.RB{i}.DrawRobot(aHandles.axes1);


                    %calculate covariance
                    sim.EKF{i}.CalculateCovariance(sim.RB{i}, DistanceAndAngle);

                    sim.EKF{i}.Draw(aHandles.axes2, sim.RB{i}.GetEstimatedPositionAndAngle);

                    sim.RB{i}.DrawTrace(aHandles.axes2);
                    if (length(landMarkFoundCell) >= 1 )

                        %sim.RB.SetEstimatedPositionAndAngle(sim.RB.GetActualPositionAndAngle());
                        %now we got all the landmarks the robot sees
                        %call ekf
                        correctedPositionAndAngle = sim.EKF{i}.ApplyFilter(sim.RB{i}, landMarkFoundCell, DistanceAndAngle);
                        sim.RB{i}.SetEstimatedPositionAndAngle(correctedPositionAndAngle) 
                    end     
                end
                pause(0.0001); 
            end   
        end
    end
end

