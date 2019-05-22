function [x,P] = flowCorrect(inputArg1,inputArg2)
%FLOWCORRECT Summary of this function goes here
%   Detailed explanation goes here
    persistent R;

    x = inputArg1;
    P = inputArg2;
end

