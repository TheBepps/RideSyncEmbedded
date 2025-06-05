function [residuals, varargout] = residuals_function(experimental, fitted)
%RESUDUALS_FUNCTION Returns a residual function

    SSE = sum((fitted-experimental).^2);
    m = mean(experimental);
    SST = sum((fitted - m).^2);
    if SST <= 0.01
        SST = 0.01;
    end

    residuals = SSE;%/SST;

    if nargout == 2
        varargout{1} = sqrt(SSE/length(experimental));
    end

end