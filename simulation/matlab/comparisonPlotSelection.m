function names = comparisonPlotSelection(scenario)
%COMPARISONPLOTSELECTION Six figures retained in version control.
switch scenario
    case 'ideal'
        names = {};
    case 'sensors'
        names = {'rmse_summary','imu_measurements'};
    case 'robustness'
        names = {'xy_tracking_comparison','rmse_summary', ...
            'tracking_error_comparison','estimation_errors'};
    otherwise
        error('comparisonPlotSelection:Scenario','Unknown scenario: %s',scenario);
end
end
