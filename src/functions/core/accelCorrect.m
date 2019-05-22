function [x,P] = accelCorrect(inputArg1,inputArg2)
%ACCELCORRECT Summary of this function goes here
%   Detailed explanation goes here
    persistent R;

    x = inputArg1;
    P = inputArg2;
end

