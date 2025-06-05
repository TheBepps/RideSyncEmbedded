function [V_oc] = open_circuit_model(seebeck_coeffs, delta_T)
%OPEN_CIRCUIT_MODEL Calculates open circuit voltage from seebeck effect
%   Given a seebeck coefficients array, it calculates the voltage output
%   from a given delta temperature
V_oc = seebeck_coeffs(1) .* delta_T;
end

