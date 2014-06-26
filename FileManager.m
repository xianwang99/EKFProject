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

classdef FileManager < handle
    %FILEMANAGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mFilePath;
    end
    
    methods
        function fm = FileManager()
        end
        
        function ReadDataFromFile(fm, sim, aFilePath)
            sim.Reset();
            fm.mFilePath = aFilePath;
            fid = fopen(aFilePath, 'r');
            lineRead = fgetl(fid);
            
            %section 1 is robot, section 2 is wp and section 3 is lm
            inSection = 0;
            while ischar(lineRead)
                if(strcmpi(lineRead, 'rb') == 1)
                    inSection = 1;
                    sim.mNbRobots = sim.mNbRobots + 1;
                elseif strcmpi(lineRead, 'wp') == 1
                    inSection = 2;
                elseif strcmpi(lineRead, 'lm') == 1
                    inSection = 3;
                elseif strcmpi(lineRead, 'axis') == 1
                    inSection = 4;
                else
                    lineRead = str2num(lineRead);
                    %add to section
                    if inSection == 1
                        sim.RB{sim.mNbRobots} = Robot(lineRead, sim.NM);
                        sim.WPM{sim.mNbRobots} = WayPointsManager(0.1);
                        sim.WPM{sim.mNbRobots}.AddWayPoint([lineRead(1), lineRead(2)]);
                        sim.EKF{sim.mNbRobots} = EKFClass(sim.NM);
                    elseif inSection == 2
                        sim.WPM{sim.mNbRobots}.AddWayPoint(lineRead);
                    elseif inSection == 3
                        sim.LMM.AddLandMark(lineRead);
                    elseif inSection == 4
                        sim.mAxis = lineRead;
                    end
                end
                lineRead = fgetl(fid);
            end
            fclose(fid);
        end
    end
end

