%% Generate many simulations with different weight matrices and model errors
noise = 0; 

k = 1;

%Loop for model error generation
    weights_OV = [0 2 0.9];
    weights_MV = [1 1 0.1];
    deltas = [0.5 1 2];
    for j = 1:length(weights_OV)
        w_ov = weights_OV(j);
        w_mv = weights_MV(j);
        for l = 1:length(deltas)
            delta= deltas(l);
            mpc_controller = MPC_setup(sys_pd,sys_kd,nu,nr,w_ov,w_mv, dt, delta);
            out = sim('single_tank_linear_real_PID.slx',sim_time);
            outs(k) = out;
            k=k+1;
        end
    end

%% Figure for disturbance results
blue = "[0 0.4470 0.7410]";
red = "[0.8500 0.3250 0.0980]";
green = "[0.4660 0.6740 0.1880]";
colors = [blue, red, green];
style = {'--' ,'-', ':'};
limits=[480 600];

figure
subplot(4,1,1)
hold on
j = 1;
for i = 1:k-1
    if mod(i,3)==1
        plot(outs(1,1).tout(400:end),outs(1,i).h(400:end),'color', colors(j),'LineStyle','--', 'LineWidth', 0.01)
    elseif mod(i,3)==2
        plot(outs(1,1).tout(400:end),outs(1,i).h(400:end),'color', colors(j),'LineStyle','-', 'LineWidth', 0.01)   
    else
        plot(outs(1,1).tout(400:end),outs(1,i).h(400:end),'color', colors(j),'LineStyle',':', 'LineWidth', 0.01)
        j = j+1;
    end
end
yline([ OV_max+h_0])
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
title('Level')
legend('MPC1, \delta = 0.5', 'MPC1, \delta = 1','MPC1, \delta = 2','MPC2, \delta = 0.5', 'MPC2, \delta = 1','MPC2, \delta = 2',...
    'MPC3, \delta = 0.5', 'MPC3, \delta = 1','MPC3, \delta = 2', 'Location','northwest')
xlim(limits)

subplot(4,1,2)
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
j = 1;
for i = 1:k-1
    if mod(i,3)==1
        plot(outs(1,1).tout(400:end),outs(1,i).u(400:end),'color', colors(j),'LineStyle','--', 'LineWidth', 0.01)
    elseif mod(i,3)==2
        plot(outs(1,1).tout(400:end),outs(1,i).u(400:end),'color', colors(j),'LineStyle','-', 'LineWidth', 0.01)   
    else
        plot(outs(1,1).tout(400:end),outs(1,i).u(400:end),'color', colors(j),'LineStyle',':', 'LineWidth', 0.01)
        j = j+1;
    end
end
title('Total control signal')
xlim(limits)


subplot(4,1,3)
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
j = 1;
for i = 1:k-1
    if mod(i,3)==1
        plot(outs(1,1).tout(400:end),outs(1,i).v(400:end),'color', colors(j),'LineStyle','--', 'LineWidth', 0.01)
    elseif mod(i,3)==2
        plot(outs(1,1).tout(400:end),outs(1,i).v(400:end),'color', colors(j),'LineStyle','-', 'LineWidth', 0.01)   
    else
        plot(outs(1,1).tout(400:end),outs(1,i).v(400:end),'color', colors(j),'LineStyle',':', 'LineWidth', 0.01)
        j = j+1;
    end
end
title('PI control signal')
xlim(limits)


subplot(4,1,4)
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
j = 1;
for i = 1:k-1
    if mod(i,3)==1
        plot(outs(1,1).tout(400:end),outs(1,i).w(400:end),'color', colors(j),'LineStyle','--', 'LineWidth', 0.01)
    elseif mod(i,3)==2
        plot(outs(1,1).tout(400:end),outs(1,i).w(400:end),'color', colors(j),'LineStyle','-', 'LineWidth', 0.01)   
    else
        plot(outs(1,1).tout(400:end),outs(1,i).w(400:end),'color', colors(j),'LineStyle',':', 'LineWidth', 0.01)
        j = j+1;
    end
end
title('MPC control signal')
xlim(limits)

sgtitle('Model errors')