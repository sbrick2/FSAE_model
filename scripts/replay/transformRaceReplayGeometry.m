function geometry = transformRaceReplayGeometry(frame, dimensions, options)
%TRANSFORMRACEREPLAYGEOMETRY Transform each wheel's Fx and Fy independently.
%   Wheel order FL FR RL RR; vectors in N; ForceScale only affects drawing.
arguments
    frame (1, 1) struct
    dimensions (1, 1) struct
    options.View (1, 1) string {mustBeMember(options.View, ...
        ["north", "heading", "overview"])} = "north"
end
front = dimensions.CGToFrontAxle;
rear = dimensions.Wheelbase - front;
wheelBody = [front, dimensions.TrackFront / 2; ...
    front, -dimensions.TrackFront / 2; ...
    -rear, dimensions.TrackRear / 2; -rear, -dimensions.TrackRear / 2];
psi = frame.Vehicle.Psi;
rotation = [cos(psi), -sin(psi); sin(psi), cos(psi)];
center = [frame.Vehicle.X, frame.Vehicle.Y];
wheel = wheelBody * rotation.' + center;
theta = psi + frame.Wheel.SteerAngle;
forceX = [frame.Tire.Fx(:) .* cos(theta(:)), ...
    frame.Tire.Fx(:) .* sin(theta(:))];
forceY = [-frame.Tire.Fy(:) .* sin(theta(:)), ...
    frame.Tire.Fy(:) .* cos(theta(:))];
bodyWidth = min(dimensions.TrackFront, dimensions.TrackRear) * 0.34;
body = [front + 0.25, 0; front * 0.75, bodyWidth; ...
    -rear - 0.25, bodyWidth; -rear - 0.25, -bodyWidth; ...
    front * 0.75, -bodyWidth];
body = body * rotation.' + center;
viewRotation = eye(2);
viewOrigin = [0, 0];
if options.View == "heading"
    angle = pi / 2 - psi;
    viewRotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    viewOrigin = center;
end
geometry = struct("WheelCenters", (wheel - viewOrigin) * viewRotation.', ...
    "Body", (body - viewOrigin) * viewRotation.', ...
    "FxVectors", forceX * viewRotation.', "FyVectors", forceY * viewRotation.', ...
    "WheelAngles", theta + atan2(viewRotation(2, 1), viewRotation(1, 1)), ...
    "Center", (center - viewOrigin) * viewRotation.', ...
    "ViewRotation", viewRotation, "ViewOrigin", viewOrigin);
end
