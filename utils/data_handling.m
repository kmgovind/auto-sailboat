function data = load_simulation_data(filename)
    % Load simulation data from a specified file
    data = load(filename);
end

function save_simulation_data(data, filename)
    % Save simulation data to a specified file
    save(filename, 'data');
end

function processed_data = process_input_parameters(params)
    % Process input parameters for simulations
    % Example: Normalize or validate parameters
    processed_data = params; % Placeholder for actual processing
end

function summary = summarize_simulation_results(results)
    % Generate a summary of simulation results
    summary.mean = mean(results);
    summary.std = std(results);
    summary.max = max(results);
    summary.min = min(results);
end