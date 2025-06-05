function [I] = current_model(coeffs,mT_Rload_Voc)
%CURRENT_MODEL Returns current from mean_temperature, load resistance and voltage
%   
mean_T = mT_Rload_Voc(:,1);
R_load = mT_Rload_Voc(:,2);
V_oc = mT_Rload_Voc(:,3);
I = V_oc ./ (R_load + internal_resistance_model(coeffs, mean_T));
end

