close all;

%% Import rosbag
bag = rosbag('hallway_corner_2020-04-15-15-27-44.bag');

%% Save wanted rosbag topics in variables
AMCL = select(bag, 'Topic', '/amcl_pose');
odom = select(bag, 'Topic', '/ropod/odom');
cmd_vel = select(bag, 'Topic', '/ropod/cmd_vel');
pred_pose_vel = select(bag, 'Topic', '/ropod/pred_pose_vel');

%% Extract data from topics into struct
AMCLStruct = readMessages(AMCL, 'DataFormat', 'struct');
x_AMCL = zeros(1,length(AMCLStruct));
y_AMCL = zeros(1,length(AMCLStruct));

odomStruct = readMessages(odom, 'DataFormat', 'struct');
x_odom = zeros(1,length(odomStruct));
y_odom = zeros(1,length(odomStruct));

cmd_velStruct = readMessages(cmd_vel, 'DataFormat', 'struct');
v_lin = zeros(1,length(cmd_velStruct));
v_ang = zeros(1,length(cmd_velStruct));

predictionsStruct = readMessages(pred_pose_vel, 'DataFormat', 'struct');
x_pred = zeros(1,length(predictionsStruct));
y_pred = zeros(1,length(predictionsStruct));
v_linPred = zeros(1,length(predictionsStruct));
v_angPred = zeros(1,length(predictionsStruct));
pred_time = zeros(1,length(predictionsStruct));
rostime = zeros(1,length(predictionsStruct));

%% Extract data from struct into vector
for i = 1:length(AMCLStruct)
    x_AMCL(i) = AMCLStruct{i,1}.Pose.Pose.Position.X;
    y_AMCL(i) = AMCLStruct{i,1}.Pose.Pose.Position.Y;
end

for i = 1:length(odomStruct)
    x_odom(i) = odomStruct{i,1}.Pose.Pose.Position.X;
    y_odom(i) = odomStruct{i,1}.Pose.Pose.Position.Y;
end

for i = 1:length(cmd_velStruct)
    v_lin(i) = cmd_velStruct{i,1}.Linear.X;
    v_ang(i) = cmd_velStruct{i,1}.Angular.Z;
end

for i = 1:length(predictionsStruct)
    x_pred(i) = predictionsStruct{i,1}.Pose.Pose.Position.X;
    y_pred(i) = predictionsStruct{i,1}.Pose.Pose.Position.Y;
    v_linPred(i) = predictionsStruct{i,1}.Twist.Twist.Linear.X;
    v_angPred(i) = predictionsStruct{i,1}.Twist.Twist.Angular.Z;
    pred_time(i) = predictionsStruct{i,1}.Twist.Twist.Linear.Y;
    rostime(i) = double(predictionsStruct{i,1}.Header.Stamp.Sec)+double(predictionsStruct{i,1}.Header.Stamp.Nsec)*10^-9;
end

init_rostime = rostime(1);

pred_time = [pred_time 0];
x_pred = [x_pred 0];    % Appending of vector with 0 value to make the last split easier
y_pred = [y_pred 0];

j = 1;
while length(pred_time) > 1        % Checking difference between two consecutive x- and y-values
    for i = 1:length(pred_time)-1  % if difference is too big, new column is started because
        time_dif = pred_time(i) - pred_time(i+1); % it belongs to a new prediction after the actual robot has moved
        rostime_dif = rostime(1) - init_rostime;
        if abs(time_dif) > 2                      
            Prediction{j,1}.Time = pred_time(1:i)+rostime_dif; 
            Prediction{j,1}.X = x_pred(1:i);
            Prediction{j,1}.Y = y_pred(1:i);
            Prediction{j,1}.v_lin = v_linPred(1:i);
            Prediction{j,1}.v_ang = v_angPred(1:i);
            pred_time(1:i) = [];
            x_pred(1:i) = [];
            y_pred(1:i) = [];
            v_linPred(1:i) = [];
            v_angPred(1:i) = [];
            rostime(1:i) = [];
            j = j+1;
            break
        end
    end
end        

for i = 1:length(v_lin)
    time_vlin(i) = 0.1*i;
end
%% Plotting
figure(1)
plot1 = plot(x_AMCL,y_AMCL, 'LineWidth', 2);
hold on
plot2 = plot(x_odom, y_odom, 'LineWidth', 2);
for i = 1:length(Prediction)
    plot3 = plot(Prediction{i,1}.X, Prediction{i,1}.Y, 'Color',[0.05,0.05,0.05]);
    plot3.Color(4) = 0.1;
end

pause(0.5)
uistack(plot2, 'top');
pause(0.5)
uistack(plot1, 'top');
hold off
axis([0 10 6.5 16])
legend([plot1, plot2, plot3], 'AMCL position', 'Odom position', 'Predicted positions')
xlabel('X position [m]')
ylabel('Y position [m]')

figure(2)
plot1 = plot(time_vlin, v_lin, 'LineWidth', 2);
hold on
for i = 1:length(Prediction)
    plot2 = plot(Prediction{i,1}.Time, Prediction{i,1}.v_lin, 'Color',[0.05,0.05,0.05]);
    plot2.Color(4) = 0.3;
end
pause(0.5)
uistack(plot1, 'top');
legend([plot1, plot2], 'Actual velocity', 'Predicted velocities')
xlabel('Time [s]')
ylabel('Velocity [m/s]')