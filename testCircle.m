%% test script (circle)

tc = tic;
t_max = 20; %s
r = 0.1;
dt = 1;
n_steps = t_max/dt;

thetha = 0:n_steps;
thetha = thetha / n_steps * 2 * pi;
inc_sum = zeros(6,1);
inc = zeros(6,1);
inc_sum(1) = r;
%fprintf('Inc: (%.2f,%.2f), Total: (%.2f,%.2f)\n',inc(1),inc(2),inc_sum(1),inc_sum(2));

%h.t_min = 0.03;
disp('Starting commands. Continue?');
%pause

for i = 2:numel(thetha)
    
    inc = zeros(6,1);
    inc(1) = cos(thetha(i)) - cos(thetha(i-1));
    inc(2) = sin(thetha(i)) - sin(thetha(i-1));
    inc = inc * r;
    
    inc_sum = inc_sum + inc;
    h.inc = inc;
    
    %fprintf('Inc: (%.2f,%.2f), Total: (%.2f,%.2f)\n',inc(1),inc(2),inc_sum(1),inc_sum(2));
    % disp('Continue?');
    pause(dt);
end

disp('Done.');
clear tc t_max r dt n_steps thetha inc inc_sum

