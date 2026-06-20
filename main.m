function main()
clear; clc; close all;

root_dir = fileparts(mfilename("fullpath"));
cd(root_dir);

% 主流程：
% 1) 先读取系统与负荷参数；
% 2) 再把小时级真实负荷加工成可用于闭环调频的外部扰动；
% 3) 最后导出结果。
system = build_system_params();
system = attach_external_regulation_profile(system);
ev = build_ev_params();
station = build_station_params();

rng(system.random_seed);
fleet = generate_ev_fleet(ev, station);

scenario_names = {"pure_thermal", "simple_dispatch", "proposed"};
scenario_results = struct();
for i = 1:numel(scenario_names)
    rng(system.random_seed + 100);
    scenario_results.(scenario_names{i}) = run_closed_loop_case(scenario_names{i}, fleet, system, ev, station);
end

sensitivity = run_sensitivity_analysis(fleet, system, ev, station);

try
    plot_results(scenario_results, station, system);
catch plot_error
    disp("绘图阶段出现错误：");
    disp(plot_error.message);
end

results = struct();
results.proposed = scenario_results.proposed;
results.scenarios = scenario_results;
results.sensitivity = sensitivity;

save("simulation_results.mat", "results", "scenario_results", "sensitivity");
export_results_to_excel("simulation_results.xlsx", scenario_results, sensitivity, station, system);
disp("仿真完成，结果已保存到 simulation_results.mat 和 simulation_results.xlsx");
end

function system = build_system_params()
system.random_seed = 42;
system.dt = 10.0;
system.sim_hours = 4.0;
system.sim_start_hour = 18.0;
system.num_steps = round(system.sim_hours * 3600 / system.dt);
system.capacity_update_interval_s = 300;
system.capacity_update_interval_steps = max(round(system.capacity_update_interval_s / system.dt), 1);
system.recovery_window_s = 60;

system.base_power_mw = 100.0;
system.nominal_frequency_hz = 50.0;
system.freq_deadband_hz = 0.02;
system.reg_kp = 58.0;
system.reg_ki = 0.55;
system.reg_ff = 0.62;
system.inertia_h = 4.2;
system.damping_d = 1.0;

system.disturbance.active_start_s = 0;
system.disturbance.ramp_duration_s = 120;
system.load_data.file_path = fullfile(fileparts(mfilename("fullpath")), "负荷数据.xlsx");
system.load_data.typical_day_method = "max_evening_mean";
system.load_data.interp_method = "pchip";
system.load_data.day_hours = 0:23;

system.residual.base_std_mw = 0.50;
system.residual.level_gain = 0.28;
system.residual.ramp_gain = 86.0;
system.residual.ar_coeff = 0.93;
system.residual.smooth_window_s = 40;
system.residual.slow_window_s = 120;
system.residual.num_pulses = 7;
system.residual.pulse_width_s = 180;
system.residual.pulse_gain = 1.08;
system.residual.target_std_mw = 2.75;
system.residual.max_abs_mw = 8.6;
system.residual.front_emphasis_hours = 2.2;
system.residual.front_emphasis_gain = 1.42;
system.residual.tail_relief_scale = 0.90;
system.residual.local_relief_windows = [ ...
    18.207, 0.020, 0.84; ...
    19.321, 0.016, 0.86; ...
    19.620, 0.044, 0.74; ...
    20.196, 0.020, 0.82; ...
    20.515, 0.016, 0.86; ...
    21.662, 0.030, 0.72];

system.thermal.initial_power = 0.0;
system.thermal.min_power = -4.5;
system.thermal.max_power = 4.5;
system.thermal.ramp_rate_mw_per_s = 0.05;
system.thermal.time_constant_s = 28.0;

system.optimization.fairness_weight = 1.4;
system.optimization.degradation_weight = 0.6;
system.optimization.smooth_weight = 2.0;
system.optimization.response_penalty = 0.8;

system.command.filter_alpha = 0.70;
system.command.hysteresis_mw = 0.01;
system.command.station_ramp_mw_per_s = 0.020;

system.simple_dispatch.effective_ratio = 0.64;
system.simple_dispatch.response_gain = 0.84;
system.simple_dispatch.filter_alpha = 0.45;
system.simple_dispatch.ramp_kw_per_vehicle = 5.5;
system.simple_dispatch.deadband_kw = 0.35;
system.simple_dispatch.dispatch_chunk_kw = 6.0;
system.simple_dispatch.min_active_vehicles = 8;
system.simple_dispatch.max_active_fraction = 0.35;

system.chance.confidence_level = 0.90;
system.chance.sigma_multiplier = 1.2816;
system.simple_uncertainty.kappa = 2.6;
system.simple_uncertainty.sigma_mult = 3.2;
system.simple_uncertainty.min_response = 0.18;
system.simple_uncertainty.max_utilization = 0.32;

system.proposed_uncertainty.kappa = 0.9;
system.proposed_uncertainty.sigma_mult = 0.6;
system.proposed_uncertainty.min_response = 0.55;
system.proposed_uncertainty.max_utilization = 0.95;

end

function system = attach_external_regulation_profile(system)
% 将小时级真实负荷转换成算例使用的净失衡输入。
% 这里不是简单复制原始负荷，而是分三步处理：
% 1) 从全年数据中提取一个典型日晚高峰负荷背景；
% 2) 叠加带相关性的快变残差与若干局部脉冲；
raw_matrix = readmatrix(system.load_data.file_path);
load_values = raw_matrix(~isnan(raw_matrix));
load_values = load_values(:);

if numel(load_values) < 24
    error("负荷数据文件中的有效数值少于24个，无法构造典型日。");
end

usable_len = floor(numel(load_values) / 24) * 24;
load_values = load_values(1:usable_len);
hourly_matrix = reshape(load_values, 24, []).';

switch system.load_data.typical_day_method
    case "max_evening_mean"
        evening_cols = 19:22; % 对应 18:00-21:00 的晚高峰小时段
        [~, typical_day_idx] = max(mean(hourly_matrix(:, evening_cols), 2));
    otherwise
        [~, typical_day_idx] = max(mean(hourly_matrix, 2));
end

typical_day_profile = hourly_matrix(typical_day_idx, :);
time_s = (0:system.num_steps - 1)' * system.dt;
hour_axis = system.sim_start_hour + time_s / 3600;

window_start = max(floor(system.sim_start_hour) - 1, 0);
window_end = min(ceil(system.sim_start_hour + system.sim_hours) + 1, 23);
support_hours = window_start:window_end;
support_cols = support_hours + 1;
support_load = typical_day_profile(support_cols);
background_load = interp1(support_hours, support_load, hour_axis, system.load_data.interp_method, "extrap");
background_load = background_load(:);
elapsed_h = time_s / 3600;

front_hours = min(system.residual.front_emphasis_hours, system.sim_hours);
disturbance_shape = ones(system.num_steps, 1);
front_idx = elapsed_h <= front_hours;
% 前半段加重扰动
disturbance_shape(front_idx) = system.residual.front_emphasis_gain;
tail_idx = ~front_idx;
if any(tail_idx)
    tail_progress = (elapsed_h(tail_idx) - front_hours) / max(system.sim_hours - front_hours, 1e-9);
    disturbance_shape(tail_idx) = 1 - ...
        (1 - system.residual.tail_relief_scale) * min(max(tail_progress, 0), 1);
end

rng(system.random_seed + 11);
level_norm = background_load / max(mean(background_load), 1e-9);
background_ramp = [0; diff(background_load)] / system.dt;
ramp_norm = abs(background_ramp) / max(max(abs(background_ramp)), 1e-9);
dynamic_sigma = system.residual.base_std_mw .* ...
    (1 + system.residual.level_gain * (level_norm - 1) + 0.8 * ramp_norm);
dynamic_sigma = max(dynamic_sigma, 0.15 * system.residual.base_std_mw);
dynamic_sigma = dynamic_sigma .* disturbance_shape;

white_noise = randn(system.num_steps, 1);
residual = zeros(system.num_steps, 1);
sqrt_term = sqrt(max(1 - system.residual.ar_coeff^2, 1e-9));
for k = 2:system.num_steps
    residual(k) = system.residual.ar_coeff * residual(k - 1) + ...
        sqrt_term * dynamic_sigma(k) * white_noise(k);
end

smooth_window_steps = max(round(system.residual.smooth_window_s / system.dt), 1);
slow_window_steps = max(round(system.residual.slow_window_s / system.dt), 1);

% 短窗平滑保留快速波动
residual = movmean(residual, smooth_window_steps);
residual = 0.7 * residual + 0.3 * movmean(residual, slow_window_steps);

pulse_signal = zeros(system.num_steps, 1);
pulse_pos = linspace(0.06, 1.0, system.residual.num_pulses) .^ 1.35;
pulse_pos = 0.05 + 0.90 * pulse_pos / max(pulse_pos);
pulse_centers = round(1 + (system.num_steps - 1) .* pulse_pos);
pulse_centers = min(max(pulse_centers, 1), system.num_steps);
pulse_width_steps = max(round(system.residual.pulse_width_s / system.dt), 1);
for i = 1:numel(pulse_centers)
    idx = (1:system.num_steps)' - pulse_centers(i);
    pulse_sign = (-1)^(i + 1);
    pulse_weight = 1 + 0.15 * double(elapsed_h(pulse_centers(i)) <= front_hours);
    pulse_signal = pulse_signal + pulse_sign * ...
        pulse_weight * system.residual.pulse_gain * dynamic_sigma(pulse_centers(i)) .* ...
        exp(-0.5 * (idx / pulse_width_steps).^2);
end

net_imbalance = residual + pulse_signal + system.residual.ramp_gain * background_ramp;
net_imbalance = net_imbalance - mean(net_imbalance);
net_std = std(net_imbalance);
if net_std > 1e-9
    net_imbalance = net_imbalance * (system.residual.target_std_mw / net_std);
end

if isfield(system.residual, "local_relief_windows") && ~isempty(system.residual.local_relief_windows)
    relief_shape = ones(system.num_steps, 1);
    for i = 1:size(system.residual.local_relief_windows, 1)
        center_h = system.residual.local_relief_windows(i, 1);
        half_width_h = system.residual.local_relief_windows(i, 2);
        scale_h = system.residual.local_relief_windows(i, 3);
        sigma_h = max(half_width_h / 2, 1e-3);
        local_weight = exp(-0.5 * ((hour_axis - center_h) / sigma_h).^2);
        relief_shape = relief_shape .* (1 - (1 - scale_h) * local_weight);
    end
 
    net_imbalance = net_imbalance .* relief_shape;
end

net_imbalance = min(max(net_imbalance, -system.residual.max_abs_mw), system.residual.max_abs_mw);

system.external_input = struct();
system.external_input.time = time_s;
system.external_input.hour_axis = hour_axis(:);
system.external_input.background_load_mw = background_load;
system.external_input.background_ramp_mw_per_s = background_ramp;
system.external_input.residual_mw = net_imbalance - system.residual.ramp_gain * background_ramp;
system.external_input.net_imbalance_mw = net_imbalance;
system.external_input.typical_day_index = typical_day_idx;
system.external_input.typical_day_profile = typical_day_profile(:);
system.external_input.data_source = system.load_data.file_path;
end

function ev = build_ev_params()
ev.battery_capacity_mean_kwh = 68;
ev.battery_capacity_std_kwh = 10;
ev.soc_min = 0.20;
ev.soc_max = 0.90;
ev.soc_target_mean = 0.78;
ev.soc_target_std = 0.06;
ev.arrival_time_mean_h = 18.5;
ev.arrival_time_std_h = 0.8;
ev.departure_time_mean_h = 7.5;
ev.departure_time_std_h = 0.6;
ev.charge_power_min_kw = -10.0;
ev.charge_power_max_kw = 10.0;
ev.charge_efficiency = 0.95;
ev.discharge_efficiency = 0.94;
ev.response_alpha = 8.0;
ev.response_beta = 2.5;
ev.participation_min = 0.45;
ev.participation_max = 0.98;
ev.degradation_cost_coeff = 0.04;
ev.fair_soc_reference = 0.60;
end

function station = build_station_params()
station.num_stations = 3;
station.vehicles_per_station = [200, 200, 200];
station.station_names = ["站1", "站2", "站3"];
station.station_power_limit_up_mw = [1.00, 1.05, 1.00];
station.station_power_limit_down_mw = [-1.00, -1.05, -1.00];
station.station_weight = [1.0, 1.05, 0.98];
end

function fleet = generate_ev_fleet(ev, station)
num_vehicles = sum(station.vehicles_per_station);
station_id = repelem(1:station.num_stations, station.vehicles_per_station);

capacity = ev.battery_capacity_mean_kwh + ev.battery_capacity_std_kwh .* randn(num_vehicles, 1);
capacity = max(capacity, 45);

soc0 = 0.35 + 0.25 * rand(num_vehicles, 1);
soc0 = min(max(soc0, ev.soc_min + 0.05), ev.soc_max - 0.10);

soc_target = ev.soc_target_mean + ev.soc_target_std .* randn(num_vehicles, 1);
soc_target = min(max(soc_target, soc0 + 0.05), ev.soc_max);

arrival_h = ev.arrival_time_mean_h + ev.arrival_time_std_h .* randn(num_vehicles, 1);
arrival_h = min(max(arrival_h, 17), 22);

departure_h = ev.departure_time_mean_h + ev.departure_time_std_h .* randn(num_vehicles, 1);
departure_h = min(max(departure_h, 6), 9);

response_prob = estimate_response_prob(num_vehicles, ev);

fleet = struct();
fleet.num_vehicles = num_vehicles;
fleet.station_id = station_id(:);
fleet.battery_capacity_kwh = capacity;
fleet.soc = soc0;
fleet.initial_soc = soc0;
fleet.target_soc = soc_target;
fleet.arrival_h = arrival_h;
fleet.departure_h = departure_h;
fleet.response_prob = response_prob;
fleet.current_power_kw = zeros(num_vehicles, 1);
fleet.reg_power_kw = zeros(num_vehicles, 1);
fleet.power_min_kw = ev.charge_power_min_kw * ones(num_vehicles, 1);
fleet.power_max_kw = ev.charge_power_max_kw * ones(num_vehicles, 1);
fleet.connected = zeros(num_vehicles, 1);
fleet.energy_gap_kwh = max((soc_target - soc0) .* capacity, 0);
end

function response_prob = estimate_response_prob(num_vehicles, ev)
g1 = randg(ev.response_alpha, num_vehicles, 1);
g2 = randg(ev.response_beta, num_vehicles, 1);
response_prob = g1 ./ max(g1 + g2, 1e-9);
response_prob = min(max(response_prob, ev.participation_min), ev.participation_max);
end

function sensitivity = run_sensitivity_analysis(fleet, system, ev, station)
levels = [0.85, 0.90, 0.95];
sigma_map = [1.036, 1.2816, 1.6449];

sensitivity = struct();
sensitivity.confidence = levels;
sensitivity.max_freq_dev = zeros(size(levels));
sensitivity.total_ev_energy = zeros(size(levels));

for k = 1:numel(levels)
    system_k = system;
    system_k.chance.confidence_level = levels(k);
    system_k.chance.sigma_multiplier = sigma_map(k);
    rng(system.random_seed + 300);
    results_k = run_closed_loop_case("proposed", fleet, system_k, ev, station);
    metrics_k = compute_case_metrics(results_k, system_k);
    sensitivity.max_freq_dev(k) = metrics_k.max_abs_freq_dev_hz;
    sensitivity.total_ev_energy(k) = metrics_k.total_ev_energy_mwh;
end
end

function metrics = compute_case_metrics(results, system)
dt_h = system.dt / 3600;

metrics = struct();
metrics.max_abs_freq_dev_hz = max(abs(results.freq_dev));
metrics.final_freq_dev_hz = results.freq_dev(end);
metrics.mean_abs_freq_dev_hz = mean(abs(results.freq_dev));
metrics.p95_abs_freq_dev_hz = prctile(abs(results.freq_dev), 95);
metrics.recovery_time_s = compute_recovery_time( ...
    results.time, results.freq_dev, system.freq_deadband_hz, ...
    system.disturbance.active_start_s, system.recovery_window_s);
metrics.violation_duration_s = compute_violation_duration( ...
    results.time, results.freq_dev, system.freq_deadband_hz, system.disturbance.active_start_s);
metrics.violation_count = count_violation_events( ...
    results.freq_dev, system.freq_deadband_hz);
metrics.total_ev_energy_mwh = sum(abs(results.ev_power)) * dt_h;
metrics.total_thermal_energy_mwh = sum(abs(results.thermal_power)) * dt_h;
metrics.peak_ev_power_mw = max(abs(results.ev_power));
metrics.peak_thermal_power_mw = max(abs(results.thermal_power));
metrics.final_ev_power_mw = results.ev_power(end);
metrics.final_thermal_power_mw = results.thermal_power(end);
metrics.mean_abs_imbalance_mw = mean(abs(results.power_imbalance));
metrics.final_imbalance_mw = results.power_imbalance(end);
metrics.avg_tracking_cost = mean(results.cost_breakdown(:, 1));
metrics.avg_fairness_cost = mean(results.cost_breakdown(:, 2));
metrics.avg_degradation_cost = mean(results.cost_breakdown(:, 3));
metrics.mean_soc_gap = mean(max(results.fleet.target_soc - results.fleet.soc, 0));
metrics.ev_energy_share = metrics.total_ev_energy_mwh / ...
    max(metrics.total_ev_energy_mwh + metrics.total_thermal_energy_mwh, 1e-9);
metrics.station_flip_count = sum(count_sign_flips(results.station_power));
end

function export_results_to_excel(file_name, scenario_results, sensitivity, station, system)
if exist(file_name, 'file')
    delete(file_name);
end

scenario_names = fieldnames(scenario_results);
summary_rows = cell(numel(scenario_names), 1);

for i = 1:numel(scenario_names)
    name = scenario_names{i};
    r = scenario_results.(name);
    metrics = compute_case_metrics(r, system);

    summary_rows{i} = struct( ...
        "scenario", string(format_scenario_name(name)), ...
        "max_abs_freq_dev_hz", metrics.max_abs_freq_dev_hz, ...
        "final_freq_dev_hz", metrics.final_freq_dev_hz, ...
        "mean_abs_freq_dev_hz", metrics.mean_abs_freq_dev_hz, ...
        "p95_abs_freq_dev_hz", metrics.p95_abs_freq_dev_hz, ...
        "recovery_time_s", metrics.recovery_time_s, ...
        "violation_duration_s", metrics.violation_duration_s, ...
        "violation_count", metrics.violation_count, ...
        "mean_soc_gap", metrics.mean_soc_gap, ...
        "total_ev_energy_mwh", metrics.total_ev_energy_mwh, ...
        "total_thermal_energy_mwh", metrics.total_thermal_energy_mwh, ...
        "ev_energy_share", metrics.ev_energy_share, ...
        "peak_ev_power_mw", metrics.peak_ev_power_mw, ...
        "peak_thermal_power_mw", metrics.peak_thermal_power_mw, ...
        "final_ev_power_mw", metrics.final_ev_power_mw, ...
        "final_thermal_power_mw", metrics.final_thermal_power_mw, ...
        "mean_abs_imbalance_mw", metrics.mean_abs_imbalance_mw, ...
        "final_imbalance_mw", metrics.final_imbalance_mw, ...
        "station_flip_count", metrics.station_flip_count, ...
        "avg_tracking_cost", metrics.avg_tracking_cost, ...
        "avg_fairness_cost", metrics.avg_fairness_cost, ...
        "avg_degradation_cost", metrics.avg_degradation_cost ...
    );

    time_table = table( ...
        r.time(:), r.disturbance.background_load(:), r.disturbance.residual(:), ...
        r.disturbance.net_imbalance(:), r.freq_dev(:), r.reg_cmd(:), ...
        r.ev_power(:), r.ev_base_power(:), r.ev_total_power(:), ...
        r.thermal_power(:), r.power_imbalance(:), ...
        'VariableNames', { ...
        'time_s', 'background_load_mw', 'residual_mw', 'disturbance_mw', ...
        'freq_dev_hz', 'reg_cmd_mw', ...
        'ev_reg_power_mw', 'ev_base_power_mw', 'ev_total_power_mw', ...
        'thermal_power_mw', 'imbalance_mw'});
    writetable(time_table, file_name, 'Sheet', make_sheet_name(['ts_' name]));

    station_table = table( ...
        r.time(:), ...
        r.station_cmd(:, 1), r.station_cmd(:, 2), r.station_cmd(:, 3), ...
        r.station_power(:, 1), r.station_power(:, 2), r.station_power(:, 3), ...
        r.station_base_power(:, 1), r.station_base_power(:, 2), r.station_base_power(:, 3), ...
        'VariableNames', {'time_s', 'cmd_s1_mw', 'cmd_s2_mw', 'cmd_s3_mw', ...
        'reg_power_s1_mw', 'reg_power_s2_mw', 'reg_power_s3_mw', ...
        'base_power_s1_mw', 'base_power_s2_mw', 'base_power_s3_mw'});
    writetable(station_table, file_name, 'Sheet', make_sheet_name(['station_' name]));
end

summary_table = struct2table([summary_rows{:}]');
writetable(summary_table, file_name, 'Sheet', 'summary');

sensitivity_table = table( ...
    sensitivity.confidence(:), sensitivity.max_freq_dev(:), sensitivity.total_ev_energy(:), ...
    'VariableNames', {'confidence', 'max_freq_dev_hz', 'total_ev_energy_mwh'});
writetable(sensitivity_table, file_name, 'Sheet', 'sensitivity');

paper_main = table( ...
    summary_table.scenario, summary_table.max_abs_freq_dev_hz, summary_table.mean_abs_freq_dev_hz, ...
    summary_table.p95_abs_freq_dev_hz, summary_table.violation_duration_s, ...
    summary_table.violation_count, summary_table.total_ev_energy_mwh, ...
    summary_table.total_thermal_energy_mwh, ...
    'VariableNames', {'scenario', 'max_abs_freq_dev_hz', 'mean_abs_freq_dev_hz', ...
    'p95_abs_freq_dev_hz', 'violation_duration_s', 'violation_count', ...
    'ev_energy_mwh', 'thermal_energy_mwh'});
writetable(paper_main, file_name, 'Sheet', 'paper_table_main');

paper_dispatch = table( ...
    summary_table.scenario, summary_table.peak_ev_power_mw, ...
    summary_table.peak_thermal_power_mw, summary_table.mean_abs_imbalance_mw, ...
    summary_table.ev_energy_share, summary_table.mean_soc_gap, ...
    summary_table.station_flip_count, ...
    'VariableNames', {'scenario', 'peak_ev_power_mw', ...
    'peak_thermal_power_mw', 'mean_abs_imbalance_mw', ...
    'ev_energy_share', 'mean_soc_gap', 'station_flip_count'});
writetable(paper_dispatch, file_name, 'Sheet', 'paper_table_dispatch');

paper_sensitivity = table( ...
    sensitivity.confidence(:), sensitivity.max_freq_dev(:), ...
    'VariableNames', {'confidence', 'max_freq_dev_hz'});
writetable(paper_sensitivity, file_name, 'Sheet', 'paper_table_sensitivity');

proposed = scenario_results.proposed;
vehicle_count = size(proposed.soc_traj, 2);
vehicle_table = table( ...
    (1:vehicle_count)', proposed.fleet.station_id(:), proposed.fleet.initial_soc(:), ...
    proposed.fleet.soc(:), proposed.fleet.target_soc(:), ...
    max(proposed.fleet.target_soc(:) - proposed.fleet.soc(:), 0), ...
    proposed.fleet.response_prob(:), proposed.fleet.arrival_h(:), ...
    proposed.fleet.departure_h(:), proposed.fleet.connected(:), ...
    proposed.fleet.battery_capacity_kwh(:), ...
    'VariableNames', {'vehicle_id', 'station_id', 'initial_soc', 'final_soc', ...
    'target_soc', 'soc_gap', 'response_prob', 'arrival_h', ...
    'departure_h', 'connected_end', 'battery_capacity_kwh'});
writetable(vehicle_table, file_name, 'Sheet', 'vehicle_summary');

capacity_table = table( ...
    cellstr(station.station_names(:)), ...
    proposed.capacity.up_min(:), proposed.capacity.up_max(:), ...
    proposed.capacity.up_sched(:), ...
    proposed.capacity.down_min(:), proposed.capacity.down_max(:), ...
    proposed.capacity.down_sched(:), ...
    proposed.capacity.response_mean(:), proposed.capacity.base_charge_mw(:), ...
    'VariableNames', {'station', 'up_min_mw', 'up_max_mw', 'up_sched_mw', ...
    'down_min_mw', 'down_max_mw', 'down_sched_mw', ...
    'mean_response_prob', 'mean_base_charge_mw'});
writetable(capacity_table, file_name, 'Sheet', 'capacity');

capacity_compare_table = table( ...
    [cellstr(station.station_names(:)); {"总计"}], ...
    [scenario_results.simple_dispatch.capacity.up_min(:); sum(scenario_results.simple_dispatch.capacity.up_min(:))], ...
    [scenario_results.proposed.capacity.up_sched(:); sum(scenario_results.proposed.capacity.up_sched(:))], ...
    100 * ([scenario_results.proposed.capacity.up_sched(:); sum(scenario_results.proposed.capacity.up_sched(:))] - ...
    [scenario_results.simple_dispatch.capacity.up_min(:); sum(scenario_results.simple_dispatch.capacity.up_min(:))]) ./ ...
    max([scenario_results.simple_dispatch.capacity.up_min(:); sum(scenario_results.simple_dispatch.capacity.up_min(:))], 1e-9), ...
    'VariableNames', {'station', 'method2_up_mw', 'proposed_up_mw', 'release_gain_pct'});
writetable(capacity_compare_table, file_name, 'Sheet', 'paper_table_capacity');

capacity_source_table = table( ...
    cellstr(station.station_names(:)), ...
    proposed.capacity.connected_count(:), proposed.capacity.mean_soc(:), ...
    proposed.capacity.response_mean(:), proposed.capacity.simple_response_factor(:), ...
    proposed.capacity.proposed_response_factor(:), proposed.capacity.release_ratio(:), ...
    proposed.capacity.mean_up_mw(:), ...
    proposed.capacity.std_up_mw(:), proposed.capacity.mean_down_mw(:), ...
    proposed.capacity.std_down_mw(:), proposed.capacity.base_charge_mw(:), ...
    'VariableNames', {'station', 'connected_ev_count', 'mean_soc', ...
    'mean_response_prob', 'simple_response_factor', 'proposed_response_factor', ...
    'release_ratio', 'mean_up_mw', 'std_up_mw', ...
    'mean_down_mw', 'std_down_mw', 'base_charge_mw'});
writetable(capacity_source_table, file_name, 'Sheet', 'capacity_source');

input_profile_table = table( ...
    proposed.disturbance.time(:), proposed.disturbance.background_load(:), ...
    proposed.disturbance.background_ramp(:), proposed.disturbance.residual(:), ...
    proposed.disturbance.net_imbalance(:), ...
    'VariableNames', {'time_s', 'background_load_mw', 'background_ramp_mw_per_s', ...
    'residual_mw', 'disturbance_mw'});
writetable(input_profile_table, file_name, 'Sheet', 'input_profile');

typical_day_table = table( ...
    (0:23)', proposed.disturbance.typical_day_profile(:), ...
    'VariableNames', {'hour_of_day', 'load_mw'});
writetable(typical_day_table, file_name, 'Sheet', 'typical_day_load');
end

function recovery_time = compute_recovery_time(time, freq_dev, deadband, step_start_s, hold_window_s)
start_idx = find(time >= step_start_s, 1, 'first');
window_len = max(round(hold_window_s / max(mean(diff(time)), 1e-9)), 1);
recovery_time = NaN;

for idx = start_idx:numel(time)
    end_idx = min(idx + window_len - 1, numel(time));
    if end_idx - idx + 1 < window_len
        break;
    end
    if all(abs(freq_dev(idx:end_idx)) <= deadband)
        recovery_time = time(idx) - step_start_s;
        return;
    end
end
end

function violation_duration = compute_violation_duration(time, freq_dev, deadband, step_start_s)
start_idx = find(time >= step_start_s, 1, 'first');
dt = mean(diff(time));
if isempty(dt) || isnan(dt)
    dt = 0;
end
violation_duration = sum(abs(freq_dev(start_idx:end)) > deadband) * dt;
end

function violation_count = count_violation_events(freq_dev, deadband)
violation_flag = abs(freq_dev(:)) > deadband;
violation_count = sum(diff([0; violation_flag]) == 1);
end

function flips = count_sign_flips(signal_matrix)
flips = zeros(1, size(signal_matrix, 2));
for i = 1:size(signal_matrix, 2)
    s = signal_matrix(:, i);
    flips(i) = sum((s(2:end) .* s(1:end-1)) < 0);
end
end

function sheet_name = make_sheet_name(raw_name)
sheet_name = char(raw_name);
sheet_name = regexprep(sheet_name, '[:\\/?*\[\]]', '_');
if strlength(string(sheet_name)) > 31
    sheet_name = extractBefore(string(sheet_name), 32);
    sheet_name = char(sheet_name);
end
end

function display_name = format_scenario_name(raw_name)
switch string(raw_name)
    case "pure_thermal"
        display_name = "方法一";
    case "simple_dispatch"
        display_name = "方法二";
    case "proposed"
        display_name = "本文方法";
    otherwise
        display_name = string(raw_name);
end
end
