function [x,P] = update(inputArg1,inputArg2)
%UPDATE Summary of this function goes here
%   Detailed explanation goes here
    persistent Q;

    x = inputArg1;
    P = inputArg2;
end
