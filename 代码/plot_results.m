function plot_results(scenarios, station, system)
output_dir = fullfile(fileparts(mfilename("fullpath")), "figures");
if ~exist(output_dir, "dir")
    mkdir(output_dir);
end

dpi = 600;
plot_frequency_comparison(scenarios, system, output_dir, dpi);
plot_power_split(scenarios.proposed, system, output_dir, dpi);
plot_capacity_release(scenarios, station, output_dir, dpi);
plot_ev_output(scenarios, system, output_dir, dpi);
plot_thermal_output(scenarios, system, output_dir, dpi);

fprintf("Figures have been exported to folder: %s\n", output_dir);
end

function plot_frequency_comparison(scenarios, system, output_dir, dpi)
hour_axis = build_hour_axis(scenarios.proposed.time, system);
fig = figure("Name", "Frequency Response Comparison", "Color", "w");
tl = tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact"); %#ok<NASGU>

nexttile;
plot(hour_axis, scenarios.pure_thermal.freq_dev, "LineWidth", 1.1); hold on;
plot(hour_axis, scenarios.simple_dispatch.freq_dev, "LineWidth", 1.1);
plot(hour_axis, scenarios.proposed.freq_dev, "LineWidth", 1.6);
plot(hour_axis, 0.02 * ones(size(hour_axis)), "--r");
plot(hour_axis, -0.02 * ones(size(hour_axis)), "--r");
grid on;
ylabel("Frequency deviation (Hz)");
title("System Frequency Responses under Different Strategies");
legend("Method 1", "Method 2", "Proposed method", ...
    "Upper limit", "Lower limit", "Location", "best");
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
xlabel("Time (h)");
ylabel("Frequency deviation (Hz)");
title("Zoomed-In View of the Limit-Exceeding Region");
xlim([zoom_center - zoom_half_width, zoom_center + zoom_half_width]);
ylim([-0.03, 0.03]);

export_figure(fig, output_dir, "Fig1_Frequency_Response", dpi);
end

function plot_power_split(results, system, output_dir, dpi)
fig = figure("Name", "EV and Thermal-Unit Regulation Sharing", "Color", "w");
window_minutes = 15;
window_steps = max(round(window_minutes * 60 / system.dt), 1);
num_bins = ceil(numel(results.time) / window_steps);

ev_energy = zeros(num_bins, 1);
thermal_energy = zeros(num_bins, 1);
center_hours = zeros(num_bins, 1);

for i = 1:num_bins
    idx_start = (i - 1) * window_steps + 1;
    idx_end = min(i * window_steps, numel(results.time));
    idx = idx_start:idx_end;

    ev_energy(i) = sum(abs(results.ev_power(idx))) * system.dt / 3600;
    thermal_energy(i) = sum(abs(results.thermal_power(idx))) * system.dt / 3600;

    center_hours(i) = system.sim_start_hour + mean(results.time(idx)) / 3600;
end

b = bar(center_hours, [thermal_energy, ev_energy], "stacked", "BarWidth", 0.012 * window_minutes);
b(1).FaceColor = [0.00, 0.45, 0.74];
b(2).FaceColor = [0.93, 0.69, 0.13];
grid on;
ylabel("Cumulative regulation energy (MWh)");
xlabel(sprintf("Time (h; accumulated every %d min)", window_minutes));
title("Cumulative Regulation Energy Sharing under the Proposed Method");
legend("Thermal-unit regulation energy", "EV regulation energy", "Location", "best");
xlim([system.sim_start_hour, system.sim_start_hour + system.sim_hours]);
xticks(system.sim_start_hour:1:(system.sim_start_hour + system.sim_hours));

export_figure(fig, output_dir, "Fig2_EV_Thermal_Power_Sharing", dpi);
end

function plot_capacity_release(scenarios, station, output_dir, dpi)
simple_up = scenarios.simple_dispatch.capacity.up_min(:);
proposed_up = scenarios.proposed.capacity.up_sched(:);

fig = figure("Name", "Available Capacity Comparison", "Color", "w");
bar_data = [simple_up, proposed_up];
station_labels = station.station_names(:);
bar(categorical(cellstr(station_labels)), bar_data, "grouped");
grid on;
ylabel("Available upward capacity (MW)");
title("Available Upward Regulation Capacity at Each Station");
legend("Method 2", "Proposed method", "Location", "best");

export_figure(fig, output_dir, "Fig3_Available_Capacity", dpi);
end

function plot_ev_output(scenarios, system, output_dir, dpi)
hour_axis = build_hour_axis(scenarios.proposed.time, system);
fig = figure("Name", "EV Regulation Output", "Color", "w");
plot(hour_axis, scenarios.simple_dispatch.ev_power, "LineWidth", 1.1); hold on;
plot(hour_axis, scenarios.proposed.ev_power, "LineWidth", 1.4);
grid on;
xlabel("Time (h)");
ylabel("Power (MW)");
title("EV Regulation Output: Method 2 vs. Proposed Method");
legend("Method 2", "Proposed method", "Location", "best");
format_hour_ticks(system);

export_figure(fig, output_dir, "Fig4_EV_Regulation_Output", dpi);
end

function plot_thermal_output(scenarios, system, output_dir, dpi)
hour_axis = build_hour_axis(scenarios.proposed.time, system);
fig = figure("Name", "Thermal-Unit Regulation Output", "Color", "w");
subplot(3, 1, 1);
plot(hour_axis, scenarios.pure_thermal.thermal_power, "LineWidth", 1.1);
grid on;
ylabel("Power (MW)");
title("Thermal-Unit Regulation Output under Method 1");
format_hour_ticks(system);

subplot(3, 1, 2);
plot(hour_axis, scenarios.simple_dispatch.thermal_power, "LineWidth", 1.1);
grid on;
ylabel("Power (MW)");
title("Thermal-Unit Regulation Output under Method 2");
format_hour_ticks(system);

subplot(3, 1, 3);
plot(hour_axis, scenarios.proposed.thermal_power, "LineWidth", 1.4);
grid on;
xlabel("Time (h)");
ylabel("Power (MW)");
title("Thermal-Unit Regulation Output under the Proposed Method");
format_hour_ticks(system);

export_figure(fig, output_dir, "Fig5_Thermal_Regulation_Output", dpi);
end

function export_figure(fig, output_dir, base_name, dpi)
png_file = fullfile(output_dir, base_name + ".png");
pdf_file = fullfile(output_dir, base_name + ".pdf");
fig_file = fullfile(output_dir, base_name + ".fig");

exportgraphics(fig, png_file, "Resolution", dpi);
exportgraphics(fig, pdf_file, "ContentType", "vector");
savefig(fig, fig_file);
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
