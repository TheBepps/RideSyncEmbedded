function [R_internal] = internal_resistance_model(coeffs, mean_T)
%INTERNAL_RESISTANCE_MODEL Calculates Internal resistance model from a mean
% temperature
%   Detailed explanation goes here
R_internal = coeffs(1) + coeffs(2) * mean_T;
end

