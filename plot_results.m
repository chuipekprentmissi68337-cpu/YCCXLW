function plot_results(scenarios, station, system)
plot_frequency_comparison(scenarios, system);
plot_power_split(scenarios.proposed, system);
plot_capacity_release(scenarios, station);
plot_ev_output(scenarios, system);
plot_thermal_output(scenarios, system);
end

function plot_frequency_comparison(scenarios, system)
hour_axis = build_hour_axis(scenarios.proposed.time, system);
figure("Name", "频率响应对比");
tl = tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(hour_axis, scenarios.pure_thermal.freq_dev, "LineWidth", 1.1); hold on;
plot(hour_axis, scenarios.simple_dispatch.freq_dev, "LineWidth", 1.1);
plot(hour_axis, scenarios.proposed.freq_dev, "LineWidth", 1.6);
plot(hour_axis, 0.02 * ones(size(hour_axis)), "--r");
plot(hour_axis, -0.02 * ones(size(hour_axis)), "--r");
grid on;
ylabel("频率偏差 / Hz");
title("不同策略下的系统频率响应");
legend("方法一", "方法二", "本文方法", ...
    "频率上限", "频率下限", "Location", "best");
format_hour_ticks(system);
y_margin = max(0.004, 1.1 * max(abs([ ...
    scenarios.pure_thermal.freq_dev; ...
    scenarios.simple_dispatch.freq_dev; ...
    scenarios.proposed.freq_dev])));
ylim([-y_margin, y_margin]);

nexttile;
[zoom_center, zoom_half_width] = choose_zoom_window(scenarios, system);
plot(hour_axis, scenarios.pure_thermal.freq_dev, "LineWidth", 1.1); hold on;
plot(hour_axis, scenarios.simple_dispatch.freq_dev, "LineWidth", 1.1);
plot(hour_axis, scenarios.proposed.freq_dev, "LineWidth", 1.6);
plot(hour_axis, 0.02 * ones(size(hour_axis)), "--r");
plot(hour_axis, -0.02 * ones(size(hour_axis)), "--r");
grid on;
xlabel("时间 / h");
ylabel("频率偏差 / Hz");
title("越限区局部放大");
xlim([zoom_center - zoom_half_width, zoom_center + zoom_half_width]);
ylim([-0.03, 0.03]);
end

function plot_power_split(results, system)
hour_axis = build_hour_axis(results.time, system);
figure("Name", "EV火电功率分担");
plot(hour_axis, results.ev_power, "LineWidth", 1.4); hold on;
plot(hour_axis, results.thermal_power, "LineWidth", 1.2);
plot(hour_axis, results.reg_cmd, "--", "LineWidth", 1.0);
grid on;
xlabel("时间 / h");
ylabel("功率 / MW");
title("本文方法下EV与火电调频功率分担");
legend("EV调频功率", "火电调频功率", "系统调频需求", "Location", "best");
format_hour_ticks(system);
end

function plot_capacity_release(scenarios, station)
simple_up = scenarios.simple_dispatch.capacity.up_min(:);
proposed_up = scenarios.proposed.capacity.up_sched(:);

figure("Name", "容量释放对比");
bar_data = [simple_up, proposed_up];
bar(categorical(cellstr(station.station_names)), bar_data, "grouped");
grid on;
ylabel("容量 / MW");
title("各站可调用上调容量对比");
legend("方法二可调容量", "本文方法可调容量", "Location", "best");
end

function plot_ev_output(scenarios, system)
hour_axis = build_hour_axis(scenarios.proposed.time, system);
figure("Name", "EV出力");
plot(hour_axis, scenarios.simple_dispatch.ev_power, "LineWidth", 1.1); hold on;
plot(hour_axis, scenarios.proposed.ev_power, "LineWidth", 1.4);
grid on;
xlabel("时间 / h");
ylabel("功率 / MW");
title("方法二与本文方法的EV调频出力对比");
legend("方法二", "本文方法", "Location", "best");
format_hour_ticks(system);
end

function plot_thermal_output(scenarios, system)
hour_axis = build_hour_axis(scenarios.proposed.time, system);
figure("Name", "火电机组出力");
plot(hour_axis, scenarios.pure_thermal.thermal_power, "LineWidth", 1.1); hold on;
plot(hour_axis, scenarios.simple_dispatch.thermal_power, "LineWidth", 1.1);
plot(hour_axis, scenarios.proposed.thermal_power, "LineWidth", 1.4);
grid on;
xlabel("时间 / h");
ylabel("功率 / MW");
title("三种策略下火电调频出力对比");
legend("方法一", "方法二", "本文方法", "Location", "best");
format_hour_ticks(system);
end

function hour_axis = build_hour_axis(time_s, system)
hour_axis = system.sim_start_hour + time_s(:) / 3600;
end

function [zoom_center, zoom_half_width] = choose_zoom_window(scenarios, system)
hour_axis = build_hour_axis(scenarios.proposed.time, system);
deadband = 0.02;
combined = max([ ...
    abs(scenarios.pure_thermal.freq_dev(:)), ...
    abs(scenarios.simple_dispatch.freq_dev(:)), ...
    abs(scenarios.proposed.freq_dev(:))], [], 2);
score = movmean(max(combined - deadband, 0), 12);
[~, idx] = max(score);
if score(idx) <= 1e-6
    [~, idx] = max(combined);
end
zoom_center = hour_axis(idx);
zoom_half_width = min(0.35, system.sim_hours / 6);
end

function format_hour_ticks(system)
xlim([system.sim_start_hour, system.sim_start_hour + system.sim_hours]);
xticks(system.sim_start_hour:1:(system.sim_start_hour + system.sim_hours));
end
