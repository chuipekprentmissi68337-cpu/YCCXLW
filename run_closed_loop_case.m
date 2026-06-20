function results = run_closed_loop_case(case_name, fleet, system, ev, station)
disturbance = generate_disturbance(system);

% 三种场景的区别只放在两个层面：
% 1) 上层容量边界是否保守；
% 2) 下层是简单分配还是站内优化调度。
switch case_name
    case "pure_thermal"
        mode.use_upper = false;
        mode.dispatch = "thermal_only";
        mode.bound_mode = "conservative";
    case "proposed"
        mode.use_upper = true;
        mode.dispatch = "admm";
        mode.bound_mode = "released";
    case "simple_dispatch"
        mode.use_upper = true;
        mode.dispatch = "simple";
        mode.bound_mode = "conservative";
    otherwise
        error("未知场景：%s", case_name);
end

nt = system.num_steps;
ns = station.num_stations;
nv = fleet.num_vehicles;

freq_dev = zeros(nt, 1);
reg_cmd = zeros(nt, 1);
thermal_power = zeros(nt, 1);
ev_power = zeros(nt, 1);
ev_base_power = zeros(nt, 1);
ev_total_power = zeros(nt, 1);
power_imbalance = zeros(nt, 1);
station_cmd = zeros(nt, ns);
station_power = zeros(nt, ns);
station_base_power = zeros(nt, ns);
soc_traj = zeros(nt, nv);
cost_breakdown = zeros(nt, 3);

fleet_state = fleet;
fleet_state = update_connection_status(fleet_state, mod(system.sim_start_hour, 24));
freq_state = struct("delta_f", 0, "integral_error", 0);
thermal_state = struct("command_power", system.thermal.initial_power, "output_power", system.thermal.initial_power);
solver_info = struct();
capacity = solve_capacity_chance_constraints(fleet_state, system, ev, station, mod(system.sim_start_hour, 24));
prev_station_cmd = zeros(1, ns);

for t = 1:nt
    current_hour = mod(system.sim_start_hour + disturbance.time(t) / 3600, 24);
    fleet_state = update_connection_status(fleet_state, current_hour);

    if mode.use_upper && (t == 1 || mod(t - 1, system.capacity_update_interval_steps) == 0)
        capacity = solve_capacity_chance_constraints(fleet_state, system, ev, station, current_hour);
    end

    freq_state.integral_error = freq_state.integral_error + system.dt * (-freq_state.delta_f);
    freq_state.integral_error = min(max(freq_state.integral_error, -0.5), 0.5);
    reg_cmd(t) = compute_regulation_command(freq_state, disturbance.net_imbalance(t), system);

    if mode.dispatch == "thermal_only"
        station_cmd(t, :) = zeros(1, ns);
        dispatch = apply_baseline_dispatch(fleet_state, system, ev, station, current_hour);
    else
        current_capacity = capacity;
        station_cmd(t, :) = project_station_command( ...
            reg_cmd(t), current_capacity, station, prev_station_cmd, system, mode.bound_mode);
        prev_station_cmd = station_cmd(t, :);

        if mode.dispatch == "admm"
            [dispatch, fleet_state, solver_info] = solve_admm_dispatch( ...
                station_cmd(t, :), fleet_state, current_capacity, system, ev, station, current_hour);
        else
            [dispatch, fleet_state] = simple_station_dispatch( ...
                station_cmd(t, :), fleet_state, system, ev, station, current_hour);
        end
    end

    station_power(t, :) = dispatch.station_reg_power;
    station_base_power(t, :) = dispatch.station_base_power;
    ev_power(t) = sum(dispatch.station_reg_power);
    ev_base_power(t) = sum(dispatch.station_base_power);
    ev_total_power(t) = sum(dispatch.station_total_power);
    soc_traj(t, :) = fleet_state.soc;

    % EV 先按站级可调用容量承担一部分调频任务，
    % 火电再去补足剩余的调频需求，因此图 3 中两者之和会尽量贴近系统指令。
    thermal_target = reg_cmd(t) - ev_power(t);
    thermal_state = thermal_dispatch_update(thermal_state, thermal_target, system);
    thermal_state = thermal_governor_model(thermal_state, system);
    thermal_power(t) = thermal_state.output_power;

    power_imbalance(t) = disturbance.net_imbalance(t) - ev_power(t) - thermal_power(t);
    freq_state = frequency_dynamics_update(freq_state, power_imbalance(t), system);
    freq_dev(t) = freq_state.delta_f;

    cost_breakdown(t, 1) = dispatch.tracking_cost;
    cost_breakdown(t, 2) = dispatch.fairness_cost;
    cost_breakdown(t, 3) = dispatch.degradation_cost;
end

results = struct();
results.case_name = case_name;
results.time = disturbance.time;
results.disturbance = disturbance;
results.freq_dev = freq_dev;
results.reg_cmd = reg_cmd;
results.ev_power = ev_power;
results.ev_base_power = ev_base_power;
results.ev_total_power = ev_total_power;
results.thermal_power = thermal_power;
results.power_imbalance = power_imbalance;
results.station_cmd = station_cmd;
results.station_power = station_power;
results.station_base_power = station_base_power;
results.soc_traj = soc_traj;
results.fleet = fleet_state;
results.capacity = capacity;
results.cost_breakdown = cost_breakdown;
results.meta.solver = solver_info;
results.meta.end_hour = mod(system.sim_start_hour + disturbance.time(end) / 3600, 24);
results.meta.charge_efficiency = ev.charge_efficiency;
end

function disturbance = generate_disturbance(system)
if ~isfield(system, "external_input")
    error("系统未提供 external_input，无法生成基于外部负荷数据的调频需求。");
end

time = system.external_input.time(:);
envelope = min(time ./ max(system.disturbance.ramp_duration_s, system.dt), 1);

disturbance = struct();
disturbance.time = time;
disturbance.background_load = system.external_input.background_load_mw(:);
disturbance.background_ramp = system.external_input.background_ramp_mw_per_s(:);
disturbance.residual = system.external_input.residual_mw(:);
disturbance.net_imbalance = envelope .* system.external_input.net_imbalance_mw(:);
disturbance.typical_day_index = system.external_input.typical_day_index;
disturbance.typical_day_profile = system.external_input.typical_day_profile(:);
end

function fleet = update_connection_status(fleet, current_hour)
if ~isfield(fleet, "reg_power_kw")
    fleet.reg_power_kw = zeros(fleet.num_vehicles, 1);
end

arrival = fleet.arrival_h;
departure = fleet.departure_h;

same_day = arrival < departure;
connected = (same_day & current_hour >= arrival & current_hour < departure) | ...
    (~same_day & (current_hour >= arrival | current_hour < departure));

fleet.connected = double(connected);
fleet.current_power_kw(~connected) = 0;
fleet.reg_power_kw(~connected) = 0;
end

function capacity = solve_capacity_chance_constraints(fleet, system, ev, station, current_hour)
ns = station.num_stations;

up_min = zeros(1, ns);
up_max = zeros(1, ns);
up_sched = zeros(1, ns);
down_min = zeros(1, ns);
down_max = zeros(1, ns);
down_sched = zeros(1, ns);
response_mean = zeros(1, ns);
release_ratio = zeros(1, ns);
simple_response_factor = zeros(1, ns);
proposed_response_factor = zeros(1, ns);
base_charge_mw = zeros(1, ns);
connected_count = zeros(1, ns);
mean_soc = zeros(1, ns);
mean_up_mw = zeros(1, ns);
std_up_mw = zeros(1, ns);
mean_down_mw = zeros(1, ns);
std_down_mw = zeros(1, ns);

% 上层容量评估是两种方法拉开差距的关键：
% 方法二不知道用户真实响应意愿，因此只能按保守下界估算可调容量；
% 本文方法利用响应概率和功率边界信息，可从同一批车辆中释放更多可用容量。
for s = 1:ns
    idx = find(fleet.station_id == s & fleet.connected > 0.5);
    if isempty(idx)
        continue;
    end

    [base_kw, ~, ~, reg_lb_kw, reg_ub_kw] = compute_vehicle_power_bounds( ...
        fleet, idx, system, ev, current_hour);
    prob = fleet.response_prob(idx);

    up_available_kw = max(reg_ub_kw, 0);
    down_available_kw = max(-reg_lb_kw, 0);
    phys_up = min(sum(up_available_kw) / 1000, station.station_power_limit_up_mw(s));
    phys_down = min(sum(down_available_kw) / 1000, abs(station.station_power_limit_down_mw(s)));

    mean_up = sum(prob .* up_available_kw) / 1000;
    std_up = sqrt(sum(prob .* (1 - prob) .* (up_available_kw / 1000).^2));
    mean_down = sum(prob .* down_available_kw) / 1000;
    std_down = sqrt(sum(prob .* (1 - prob) .* (down_available_kw / 1000).^2));
    prob_mean = mean(prob);
    prob_std = std(prob);

    connected_count(s) = numel(idx);
    mean_soc(s) = mean(fleet.soc(idx));
    response_mean(s) = prob_mean;
    base_charge_mw(s) = -sum(base_kw) / 1000;
    mean_up_mw(s) = mean_up;
    std_up_mw(s) = std_up;
    mean_down_mw(s) = mean_down;
    std_down_mw(s) = std_down;

    up_max(s) = phys_up;
    down_max(s) = -phys_down;

    simple_response_factor(s) = max( ...
        prob_mean - system.simple_uncertainty.kappa * prob_std, ...
        system.simple_uncertainty.min_response);
    simple_response_factor(s) = min(simple_response_factor(s), 1.0);
    up_simple_mean = simple_response_factor(s) * phys_up;
    down_simple_mean = simple_response_factor(s) * phys_down;
    up_min(s) = max(up_simple_mean - system.simple_uncertainty.sigma_mult * std_up, 0);
    up_min(s) = min(up_min(s), system.simple_uncertainty.max_utilization * phys_up);
    down_simple_abs = max(down_simple_mean - system.simple_uncertainty.sigma_mult * std_down, 0);
    down_simple_abs = min(down_simple_abs, system.simple_uncertainty.max_utilization * phys_down);
    down_min(s) = -down_simple_abs;

    proposed_response_factor(s) = min(max( ...
        prob_mean + system.proposed_uncertainty.kappa * prob_std, ...
        system.proposed_uncertainty.min_response), 1.0);
    up_prop_mean = proposed_response_factor(s) * phys_up;
    down_prop_mean = proposed_response_factor(s) * phys_down;
    up_sched(s) = max(up_prop_mean - system.proposed_uncertainty.sigma_mult * std_up, 0);
    up_sched(s) = min(up_sched(s), system.proposed_uncertainty.max_utilization * phys_up);
    up_sched(s) = max(up_sched(s), up_min(s));
    down_sched_abs = max(down_prop_mean - system.proposed_uncertainty.sigma_mult * std_down, 0);
    down_sched_abs = min(down_sched_abs, system.proposed_uncertainty.max_utilization * phys_down);
    down_sched(s) = -down_sched_abs;
    down_sched(s) = min(down_sched(s), down_min(s));
    down_sched(s) = max(down_sched(s), down_max(s));
    release_ratio(s) = max( ...
        (up_sched(s) - up_min(s)) / max(up_max(s) - up_min(s), 1e-9), 0);
end

capacity = struct();
capacity.up_min = up_min;
capacity.up_max = up_max;
capacity.up_sched = up_sched;
capacity.down_min = down_min;
capacity.down_max = down_max;
capacity.down_sched = down_sched;
capacity.response_mean = response_mean;
capacity.release_ratio = release_ratio;
capacity.simple_response_factor = simple_response_factor;
capacity.proposed_response_factor = proposed_response_factor;
capacity.base_charge_mw = base_charge_mw;
capacity.connected_count = connected_count;
capacity.mean_soc = mean_soc;
capacity.mean_up_mw = mean_up_mw;
capacity.std_up_mw = std_up_mw;
capacity.mean_down_mw = mean_down_mw;
capacity.std_down_mw = std_down_mw;
end

function station_cmd = project_station_command(reg_cmd, capacity, station, prev_cmd, system, bound_mode)
ns = station.num_stations;
weights = station.station_weight(:)' ./ sum(station.station_weight);
raw_cmd = reg_cmd .* weights;

if reg_cmd >= 0
    lb = zeros(1, ns);
    switch bound_mode
        case "released"
            ub = capacity.up_sched;
        otherwise
            ub = capacity.up_min;
    end
else
    switch bound_mode
        case "released"
            lb = capacity.down_sched;
        otherwise
            lb = capacity.down_min;
    end
    ub = zeros(1, ns);
end

raw_cmd = redistribute_with_bounds(raw_cmd, reg_cmd, lb, ub);
alpha = system.command.filter_alpha;
% 站级指令先做一次滤波和爬坡限制，避免场站功率指令频繁突变。
station_cmd = alpha .* prev_cmd + (1 - alpha) .* raw_cmd;

max_step = system.command.station_ramp_mw_per_s * system.dt;
delta_cmd = station_cmd - prev_cmd;
delta_cmd = min(max(delta_cmd, -max_step), max_step);
station_cmd = prev_cmd + delta_cmd;

small_idx = abs(station_cmd) < system.command.hysteresis_mw;
station_cmd(small_idx) = 0;

for s = 1:ns
    if prev_cmd(s) * station_cmd(s) < 0 && abs(station_cmd(s)) < 2 * system.command.hysteresis_mw
        station_cmd(s) = 0;
    end
end

station_cmd = redistribute_with_bounds(station_cmd, reg_cmd, lb, ub);
end

function station_cmd = redistribute_with_bounds(station_cmd, total_cmd, lb, ub)
station_cmd = min(max(station_cmd, lb), ub);
residual = total_cmd - sum(station_cmd);

for k = 1:6
    if abs(residual) < 1e-6
        break;
    end
    if residual >= 0
        room = max(ub - station_cmd, 0);
        active = room > 1e-8;
        if ~any(active)
            break;
        end
        share = room(active) ./ max(sum(room(active)), 1e-9);
        station_cmd(active) = station_cmd(active) + residual .* share;
    else
        room = max(station_cmd - lb, 0);
        active = room > 1e-8;
        if ~any(active)
            break;
        end
        share = room(active) ./ max(sum(room(active)), 1e-9);
        station_cmd(active) = station_cmd(active) - abs(residual) .* share;
    end
    station_cmd = min(max(station_cmd, lb), ub);
    residual = total_cmd - sum(station_cmd);
end
end

function [dispatch, fleet] = apply_baseline_dispatch(fleet, system, ev, station, current_hour)
ns = station.num_stations;
station_reg_power = zeros(1, ns);
station_base_power = zeros(1, ns);
station_total_power = zeros(1, ns);
fairness_cost = 0;
degradation_cost = 0;

for s = 1:ns
    idx = find(fleet.station_id == s & fleet.connected > 0.5);
    if isempty(idx)
        continue;
    end

    % 方法一中 EV 不参与调频，只按离站前补能需求执行基线充电。
    [base_kw, ~, ~, ~, ~] = compute_vehicle_power_bounds(fleet, idx, system, ev, current_hour);
    fleet = apply_vehicle_power(fleet, idx, base_kw, zeros(numel(idx), 1), system, ev);
    station_base_power(s) = sum(base_kw) / 1000;
    station_total_power(s) = station_base_power(s);
    fairness_cost = fairness_cost + sum((fleet.soc(idx) - mean(fleet.soc(idx))).^2);
    degradation_cost = degradation_cost + ev.degradation_cost_coeff * sum(abs(base_kw)) * system.dt / 3600;
end

dispatch = struct();
dispatch.station_reg_power = station_reg_power;
dispatch.station_base_power = station_base_power;
dispatch.station_total_power = station_total_power;
dispatch.tracking_cost = 0;
dispatch.fairness_cost = fairness_cost;
dispatch.degradation_cost = degradation_cost;
end

function qp = build_local_qp(cmd_mw, fleet, idx, system, ev, current_hour)
cmd_kw = 1000 * cmd_mw;
soc = fleet.soc(idx);
response = fleet.response_prob(idx);
prev_reg = fleet.reg_power_kw(idx);

[base_kw, ~, ~, reg_lb_kw, reg_ub_kw] = compute_vehicle_power_bounds( ...
    fleet, idx, system, ev, current_hour);

w2 = system.optimization.fairness_weight;
w3 = system.optimization.degradation_weight;
w4 = system.optimization.smooth_weight;
w5 = system.optimization.response_penalty;

soc_bias = ev.fair_soc_reference - soc;
diag_h = 2 * (w3 + w4 + w5 ./ max(response, 0.15) + 1e-6);
f = 2 * w2 * soc_bias - 2 * w4 * prev_reg;

qp = struct();
qp.Hdiag = diag_h(:);
qp.f = f(:);
qp.Aeq = ones(1, numel(idx));
qp.beq = cmd_kw;
qp.lb = reg_lb_kw(:);
qp.ub = reg_ub_kw(:);
qp.base_kw = base_kw(:);
qp.prev_reg = prev_reg(:);
end

function [dispatch, fleet, info] = solve_admm_dispatch(station_cmd, fleet, capacity, system, ev, station, current_hour)
ns = station.num_stations;
station_reg_power = zeros(1, ns);
station_base_power = zeros(1, ns);
station_total_power = zeros(1, ns);
tracking_cost = 0;
fairness_cost = 0;
degradation_cost = 0;
solver_status = strings(ns, 1);

for s = 1:ns
    idx = find(fleet.station_id == s & fleet.connected > 0.5);
    if isempty(idx)
        continue;
    end

    % 对每个站内车辆求一个局部二次规划，使总出力贴近站级指令，
    % 同时兼顾 SOC 公平性、动作平滑性和退化代价。
    qp = build_local_qp(station_cmd(s), fleet, idx, system, ev, current_hour);
    [delta_kw, status] = solve_exact_local_qp(qp);
    total_kw = qp.base_kw + delta_kw;

    fleet = apply_vehicle_power(fleet, idx, total_kw, delta_kw, system, ev);
    station_reg_power(s) = sum(delta_kw) / 1000;
    station_base_power(s) = sum(qp.base_kw) / 1000;
    station_total_power(s) = sum(total_kw) / 1000;
    tracking_cost = tracking_cost + (station_reg_power(s) - station_cmd(s))^2;
    fairness_cost = fairness_cost + sum((fleet.soc(idx) - mean(fleet.soc(idx))).^2);
    degradation_cost = degradation_cost + ev.degradation_cost_coeff * sum(abs(delta_kw)) * system.dt / 3600;
    solver_status(s) = status;
end

dispatch = struct();
dispatch.station_reg_power = station_reg_power;
dispatch.station_base_power = station_base_power;
dispatch.station_total_power = station_total_power;
dispatch.tracking_cost = tracking_cost;
dispatch.fairness_cost = fairness_cost;
dispatch.degradation_cost = degradation_cost;

info = struct();
info.method = "exact_station_qp";
info.capacity = capacity;
info.status = solver_status;
end

function [dispatch, fleet] = simple_station_dispatch(station_cmd, fleet, system, ev, station, current_hour)
ns = station.num_stations;
station_reg_power = zeros(1, ns);
station_base_power = zeros(1, ns);
station_total_power = zeros(1, ns);
tracking_cost = 0;
fairness_cost = 0;
degradation_cost = 0;

for s = 1:ns
    idx = find(fleet.station_id == s & fleet.connected > 0.5);
    if isempty(idx)
        continue;
    end

    [base_kw, ~, ~, reg_lb_kw, reg_ub_kw] = compute_vehicle_power_bounds( ...
        fleet, idx, system, ev, current_hour);
    cmd_kw = 1000 * station_cmd(s);
    prev_reg_kw = fleet.reg_power_kw(idx);
    desired_kw = zeros(numel(idx), 1);

    up_flex_kw = max(reg_ub_kw, 0);
    down_flex_kw = max(-reg_lb_kw, 0);
    if cmd_kw >= 0
        dir_flex_kw = up_flex_kw;
    else
        dir_flex_kw = down_flex_kw;
    end

    max_active = max( ...
        system.simple_dispatch.min_active_vehicles, ...
        round(system.simple_dispatch.max_active_fraction * numel(idx)));
    active_count = max( ...
        system.simple_dispatch.min_active_vehicles, ...
        ceil(abs(cmd_kw) / max(system.simple_dispatch.dispatch_chunk_kw, 1e-9)));
    active_count = min(active_count, min(max_active, numel(idx)));

    if abs(cmd_kw) > 1e-6 && any(dir_flex_kw > 1e-6) && active_count > 0
        % 简单策略不解优化问题，只挑选一部分响应概率高、裕度大的车辆近似承担指令。
        score = fleet.response_prob(idx) .* ...
            (0.35 + dir_flex_kw ./ max(max(dir_flex_kw), 1e-9));
        [~, order] = sort(score, "descend");
        active_idx = order(1:active_count);

        weights = zeros(numel(idx), 1);
        weights(active_idx) = fleet.response_prob(idx(active_idx)) .* ...
            max(dir_flex_kw(active_idx), 0.2);
        weights = weights / max(sum(weights), 1e-9);

        achieved_cmd_kw = system.simple_dispatch.effective_ratio * cmd_kw;
        desired_kw(active_idx) = system.simple_dispatch.response_gain * achieved_cmd_kw .* ...
            weights(active_idx);
    end

    delta_kw = system.simple_dispatch.filter_alpha .* prev_reg_kw + ...
        (1 - system.simple_dispatch.filter_alpha) .* desired_kw;
    max_delta_kw = system.simple_dispatch.ramp_kw_per_vehicle;
    delta_kw = min(max(delta_kw, prev_reg_kw - max_delta_kw), prev_reg_kw + max_delta_kw);
    small_idx = abs(delta_kw) < system.simple_dispatch.deadband_kw;
    delta_kw(small_idx) = 0;
    reverse_idx = prev_reg_kw .* desired_kw < 0 & abs(desired_kw) < 2 * system.simple_dispatch.deadband_kw;
    delta_kw(reverse_idx) = 0;
    delta_kw = min(max(delta_kw, reg_lb_kw), reg_ub_kw);
    total_kw = base_kw + delta_kw;

    fleet = apply_vehicle_power(fleet, idx, total_kw, delta_kw, system, ev);
    station_reg_power(s) = sum(delta_kw) / 1000;
    station_base_power(s) = sum(base_kw) / 1000;
    station_total_power(s) = sum(total_kw) / 1000;
    tracking_cost = tracking_cost + (station_reg_power(s) - station_cmd(s))^2;
    fairness_cost = fairness_cost + sum((fleet.soc(idx) - mean(fleet.soc(idx))).^2);
    degradation_cost = degradation_cost + ev.degradation_cost_coeff * sum(abs(delta_kw)) * system.dt / 3600;
end

dispatch = struct();
dispatch.station_reg_power = station_reg_power;
dispatch.station_base_power = station_base_power;
dispatch.station_total_power = station_total_power;
dispatch.tracking_cost = tracking_cost;
dispatch.fairness_cost = fairness_cost;
dispatch.degradation_cost = degradation_cost;
end

function [base_kw, total_lb_kw, total_ub_kw, reg_lb_kw, reg_ub_kw] = compute_vehicle_power_bounds(fleet, idx, system, ev, current_hour)
dt_h = system.dt / 3600;
cap = fleet.battery_capacity_kwh(idx);
soc = fleet.soc(idx);
energy_gap_kwh = max((fleet.target_soc(idx) - soc) .* cap, 0);
time_to_departure_h = compute_time_to_departure(current_hour, fleet.departure_h(idx));
time_to_departure_h = max(time_to_departure_h, dt_h);

baseline_charge_kw = energy_gap_kwh ./ max(ev.charge_efficiency .* time_to_departure_h, 1e-9);
baseline_charge_kw = min(baseline_charge_kw, fleet.power_max_kw(idx));
base_kw = -baseline_charge_kw;

charge_room_kw = max((ev.soc_max - soc) .* cap ./ max(ev.charge_efficiency * dt_h, 1e-9), 0);
discharge_room_kw = max((soc - ev.soc_min) .* cap .* ev.discharge_efficiency ./ dt_h, 0);
total_lb_kw = max(fleet.power_min_kw(idx), -charge_room_kw);
total_ub_kw = min(fleet.power_max_kw(idx), discharge_room_kw);

remaining_after_h = max(time_to_departure_h - dt_h, 0);
recoverable_gap_kwh = remaining_after_h .* fleet.power_max_kw(idx) .* ev.charge_efficiency;
future_discharge_limit_kw = max((recoverable_gap_kwh - energy_gap_kwh) .* ev.discharge_efficiency ./ dt_h, 0);
total_ub_kw = min(total_ub_kw, future_discharge_limit_kw);

base_kw = min(max(base_kw, total_lb_kw), min(total_ub_kw, 0));
reg_lb_kw = total_lb_kw - base_kw;
reg_ub_kw = total_ub_kw - base_kw;
end

function [x, status] = solve_exact_local_qp(qp)
if isempty(qp.f)
    x = zeros(0, 1);
    status = "empty";
    return;
end

min_sum = qp.Aeq * qp.lb;
max_sum = qp.Aeq * qp.ub;
beq_eff = min(max(qp.beq, min_sum), max_sum);

if abs(beq_eff - min_sum) < 1e-9
    x = qp.lb;
    status = "at_lower_bound";
    return;
elseif abs(beq_eff - max_sum) < 1e-9
    x = qp.ub;
    status = "at_upper_bound";
    return;
end

g = @(lambda) qp.Aeq * min(max((-(qp.f + lambda * qp.Aeq')) ./ qp.Hdiag, qp.lb), qp.ub) - beq_eff;
low = -1;
high = 1;

while g(low) < 0
    low = 2 * low;
    if abs(low) > 1e8
        break;
    end
end
while g(high) > 0
    high = 2 * high;
    if abs(high) > 1e8
        break;
    end
end

for iter = 1:80
    mid = 0.5 * (low + high);
    if g(mid) > 0
        low = mid;
    else
        high = mid;
    end
end

lambda = 0.5 * (low + high);
x = (-(qp.f + lambda * qp.Aeq')) ./ qp.Hdiag;
x = min(max(x, qp.lb), qp.ub);
x = distribute_residual(x, beq_eff, qp.lb, qp.ub);
status = "optimal";
end

function x = distribute_residual(x, target_sum, lb, ub)
residual = target_sum - sum(x);
for k = 1:8
    if abs(residual) < 1e-7
        break;
    end
    if residual > 0
        room = ub - x;
    else
        room = lb - x;
    end
    active = abs(room) > 1e-8;
    if ~any(active)
        break;
    end
    share = abs(room(active)) ./ max(sum(abs(room(active))), 1e-9);
    x(active) = x(active) + residual .* share .* sign(room(active));
    x = min(max(x, lb), ub);
    residual = target_sum - sum(x);
end
end

function fleet = apply_vehicle_power(fleet, idx, total_kw, reg_kw, system, ev)
fleet.current_power_kw(idx) = total_kw;
fleet.reg_power_kw(idx) = reg_kw;

delta_soc = zeros(size(total_kw));
discharge_idx = total_kw >= 0;
delta_soc(discharge_idx) = -(system.dt / 3600) .* total_kw(discharge_idx) ./ ...
    (fleet.battery_capacity_kwh(idx(discharge_idx)) * ev.discharge_efficiency);
delta_soc(~discharge_idx) = -(system.dt / 3600) .* total_kw(~discharge_idx) .* ...
    ev.charge_efficiency ./ fleet.battery_capacity_kwh(idx(~discharge_idx));

fleet.soc(idx) = min(max(fleet.soc(idx) + delta_soc, ev.soc_min), ev.soc_max);
fleet.energy_gap_kwh(idx) = max((fleet.target_soc(idx) - fleet.soc(idx)) .* fleet.battery_capacity_kwh(idx), 0);
end

function time_to_departure_h = compute_time_to_departure(current_hour, departure_h)
time_to_departure_h = departure_h - current_hour;
time_to_departure_h(time_to_departure_h <= 0) = time_to_departure_h(time_to_departure_h <= 0) + 24;
end

function thermal_state = thermal_governor_model(thermal_state, system)
alpha = system.dt / max(system.thermal.time_constant_s, system.dt);
output_power = thermal_state.output_power + alpha * (thermal_state.command_power - thermal_state.output_power);
thermal_state.output_power = min(max(output_power, system.thermal.min_power), system.thermal.max_power);
end

function thermal_state = thermal_dispatch_update(thermal_state, target_power, system)
delta = target_power - thermal_state.command_power;
max_delta = system.thermal.ramp_rate_mw_per_s * system.dt;
delta = min(max(delta, -max_delta), max_delta);
thermal_state.command_power = thermal_state.command_power + delta;
thermal_state.command_power = min(max(thermal_state.command_power, system.thermal.min_power), system.thermal.max_power);
end

function reg_cmd = compute_regulation_command(freq_state, disturbance_mw, system)
freq_error = -freq_state.delta_f;
if abs(freq_state.delta_f) <= system.freq_deadband_hz
    freq_error = 0.5 * freq_error;
end
reg_cmd = system.reg_ff * disturbance_mw + ...
    system.reg_kp * freq_error + ...
    system.reg_ki * freq_state.integral_error;
end

function state = frequency_dynamics_update(state, power_imbalance_mw, system)
base_term = system.base_power_mw * system.damping_d;
den = max(2 * system.inertia_h * system.base_power_mw, 1e-6);
dfdt = -(power_imbalance_mw + base_term * state.delta_f) / den;
state.delta_f = state.delta_f + system.dt * dfdt;
end
