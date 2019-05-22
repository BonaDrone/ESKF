function [x,P] = rangeCorrect(inputArg1,inputArg2)
%RANGECORRECT Summary of this function goes here
%   Detailed explanation goes here
    persistent R;

    x = inputArg1;
    P = inputArg2;
end

