initial_pos = [0; 1];
v_max = 2;
a_max = 2;

load testline
time = t;
cum_error = zeros(length(time),1);
cum_error(1) = 0;
for i = 1:length(time)
    t = time(i);
    if t <= v_max/a_max
        dt = t;
        acc = [a_max; 0];
        vel = acc*dt;
        pos = 0.5*acc*dt.^2;
        jerk = [0;0];
        snap = [0;0];
    elseif t <= 2*v_max/a_max
        dt = t - v_max/a_max;
        acc = [0; 0];
        vel = [v_max; 0];
        pos = [v_max.^2/(2*a_max);0] + [v_max*dt; 0];
        jerk = [0;0];
        snap = [0;0];
    elseif t <= 3*v_max/a_max
        dt = t - 2*v_max/a_max;
        acc = [-a_max; 0];
        vel = [v_max; 0] + acc*dt;
        pos = [3*v_max.^2/(2*a_max); 0] + [v_max;0]*dt + 0.5*acc*dt.^2;
        jerk = [0;0];
        snap = [0;0];
    else
        acc = [0;0];
        vel = [0;0];
        pos = [2*v_max.^2/a_max; 0];
        jerk = [0;0];
        snap = [0;0];
    end
    
    desired_state.pos(:,i) = initial_pos + pos;
    desired_state.vel(:,i) = vel;
    desired_state.acc(:,i) = acc;
    desired_state.jerk(:,i) = jerk;
    desired_state.snap(:,i) = snap;
    
    e = desired_state.pos(:,i) - state(i,1:2)';
    pos_error = sqrt(e'*e);
    if (i ~= length(time))
        cum_error(i+1) = cum_error(i)+pos_error*(time(i+1)-time(i));
    end
end

subplot(2,1,1)
plot(time,desired_state.pos(1,:),time,state(:,1),'r*')
subplot(2,1,2)
plot(time,cum_error);