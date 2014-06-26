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

classdef LandMarksManager < handle
    %Manages Landmarks
    
    properties
        %cell containing all landmarks and their position
        mLandMarksCell;
        
        %amount of landmarks
        mNbLandMarks=0;
    end
    
    methods
        %constructor default
        function lm = LandMarksManager()

        end
        
        %position is [x, y]
        function AddLandMark(lm, aPosition)
            lm.mNbLandMarks = lm.mNbLandMarks+1;
            lm.mLandMarksCell{lm.mNbLandMarks} = aPosition;
        end
        
        function landMark = GetLandMark(lm, aIndex )
            landMark = lm.mLandMarksCell{aIndex};
        end
        
        function nbLandMarks = GetNbLandMarks (lm)
            nbLandMarks = lm.mNbLandMarks;
        end
        
        function Draw(lm, aAxes)
            hold on;
            for i = 1 : lm.mNbLandMarks
                plot(aAxes, lm.mLandMarksCell{i}(1), lm.mLandMarksCell{i}(2), 'k*');
            end
            hold off;
        end
    end
    
end

