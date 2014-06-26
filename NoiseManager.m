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

classdef NoiseManager < handle

    properties
        mRightWheelNoise=0.01;
        mLeftWheelNoise=0.01;
        mAngleNoise=0.5*pi/180;
        mScannerAngleNoise=0.048;
        mScannerMeasureNoise=0.01;
        mSystemNoise=0.01;
    end
    
    methods
        function NM = NoiseManager()
            
        end
        
        function RandomNoise = GenerateLeftWheelNoise(NM)
            
            RandomNoise = NM.mLeftWheelNoise * rand;
            if rand > 0.5
                RandomNoise = RandomNoise * -1;
            end
        end
        
        function RandomNoise = GenerateRightWheelNoise(NM)
            
            RandomNoise = NM.mRightWheelNoise * rand;
            if rand > 0.5
                RandomNoise = RandomNoise * -1;
            end
        end
        
        function RandomNoise = GenerateAngleNoise(NM)
            RandomNoise = NM.mAngleNoise * rand;
            if rand > 0.5
                RandomNoise = RandomNoise * -1;
            end
        end
        
        function RandomNoise = GenerateScannerAngleNoise(NM)
            RandomNoise = NM.mScannerAngleNoise * rand;
            if rand > 0.5
                RandomNoise = RandomNoise * -1;
            end
        end
        
        function RandomNoise = GenerateScannerMeasureNoise(NM)
            RandomNoise = NM.mScannerMeasureNoise * rand;
            if rand > 0.5
                RandomNoise = RandomNoise * -1;
            end
        end
    end
end

