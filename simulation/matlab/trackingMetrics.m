function metrics = trackingMetrics(out, p, rampTime)
%TRACKINGMETRICS Physical-distance errors and explicitly defined effort.
error = out.X(1:3,:)-out.Ref(1:3,:);
distance = vecnorm(error,2,1);
circle = out.t >= rampTime;
metrics.rmse3D = sqrt(mean(distance.^2));
metrics.rmseCoordinate = sqrt(mean(error(:).^2));
metrics.rmsePerAxis = sqrt(mean(error.^2,2))';
metrics.rmseCircle3D = sqrt(mean(distance(circle).^2));
metrics.maxError3D = max(distance);
metrics.meanCommandDeviation = mean(vecnorm(out.U-p.uHover,2,1));
metrics.peakCommandStep = max(abs(diff([p.uHover*ones(4,1),out.U],1,2)),[],'all')/p.uHover;
metrics.saturatedSamples = nnz(out.saturated);
metrics.rateLimitedSamples = nnz(out.rateLimited);
metrics.meanSolveTimeMs = 1000*mean(out.solveTime);
metrics.maxSolveTimeMs = 1000*max(out.solveTime);
estimateError = out.XEstimated-out.X;
estimateError(4:6,:) = atan2(sin(estimateError(4:6,:)),cos(estimateError(4:6,:)));
metrics.estimationPositionRMSE = sqrt(mean(sum(estimateError(1:3,:).^2,1)));
metrics.estimationAttitudeRMSEDeg = rad2deg(sqrt(mean(sum(estimateError(4:6,:).^2,1))));
end
