%% Data collection
data = read_arduino_serial("COM4",115200);

%% Animation of the position and orientation of a vehicle
% define some data to animate
% first column is time, second is x, thrid is y, fourth is phi
time = data(:,1);
xposition = data(:,2);
yposition = data(:,3);
phi = data(:,4);

% Define shape verticies
% first row is x position, second row is y position
r_width = .5;
r_length = 1;
V = [-r_length/2 -r_length/2 0 r_length/2 0;
-r_width/2 r_width/2 r_width/2 0 -r_width/2];

figure
for i=1:length(time)
    % move shape by moving verticies
    % define rotation matrix
    T = [cos(phi(i)) -sin(phi(i));sin(phi(i)) cos(phi(i))];
    % define center position
    pos = [xposition(i);yposition(i)];
    % find position of current vertices: each position is multiplied by the rotation
    %matrix, and added to the current position
    v_c = T*V+pos*ones(1,5);
    % draw shape
    fill(v_c(1,:),v_c(2,:),'y')
    axis([-3 3 -3 3]) % set axis to have specified x and y limits
    % (type 'help axis' for more info)
    % make sure matlab draws the figure now
    drawnow
    % if not last drawing, wait
    if i<length(time)
        pause((time(i+1)-time(i)) / 100)
    end
end