function myplot(path)
%     path = [2.4 0.6 0;
%         7.1 0.8 0;
%         10.2 0.9 0;
%         13.5 0.8 0;
%         16.5 0.9 0;
%         19.4 1 0;
%         22.7 1.2 0;
%         27 1.1 0;
%         30.3 1.1 0;
%         33.4 0.8 0;
%         36.6 0.8 0;
%         39.4 0.9 0;
%         42.7 0.8 0;
%         47 0.6 0;
%         51.2 0.4 0;
%         54.7 0.6 0];
% 
    path = path(:,1:2);
    time = 1:0.1:20;

    initial_location = path(1,:);
    final_location = path(end,:);
    initial_orientation = 0;
    current_pose = [initial_location initial_orientation]';

    model = differentialDriveKinematics("TrackWidth",1,"VehicleInputs","VehicleSpeedHeadingRate");

    figure
    plot(path(:,1), path(:,2),'k--d')
    xlim([0 80])
    ylim([0 20])

    controller = controllerPurePursuit;

    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 3;
    controller.MaxAngularVelocity = 2;
    controller.LookaheadDistance = 0.8;

    goal_radius = 1;
    distance_left = norm(final_location - initial_location);

    sample_time = 0.1;
    rate = rateControl(1/sample_time);

    figure
    frame = model.TrackWidth/0.8;

    while (distance_left > goal_radius)

        [v,omega] = controller(current_pose);
        vel = derivative(model, current_pose, [v omega]);
        current_pose = current_pose + vel*sample_time;
        distance_left = norm(current_pose(1:2) - final_location(:));

        hold off

        plot(path(:,1), path(:,2),"k--d")
        hold all

        vec = [current_pose(1:2); 0];
        rot = axang2quat([0 0 1 current_pose(3)]);
        plotTransforms(vec', rot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frame);
        light;
        xlim([0 80])
        ylim([0 20])
        waitfor(rate);
    end