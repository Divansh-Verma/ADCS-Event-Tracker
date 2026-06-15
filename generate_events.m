function [new_events, current_state] = generate_events(curr_u, curr_v, curr_ids, prev_state, t_current)
    % generate_events Creates asynchronous events from moving star coordinates.
    %
    % Inputs:
    % curr_u, curr_v: Current pixel coordinates (floats)
    % curr_ids: Array of visible star IDs
    % prev_state: Struct containing the pixel locations from the previous time step
    % t_current: Current simulation time
    %
    % Outputs:
    % new_events: N x 4 matrix where each row is [x_pixel, y_pixel, timestamp, polarity]
    % current_state: Struct to pass into the next iteration as prev_state

    % Quantize float coordinates to discrete integer pixel bins
    curr_u_int = round(curr_u);
    curr_v_int = round(curr_v);
    
    new_events = []; % Initialize empty event array
    
    % If there is a previous state, compare current against it
    if ~isempty(prev_state)
        prev_ids = prev_state.ids;
        prev_u_int = prev_state.u;
        prev_v_int = prev_state.v;
        
        % Check for stars that moved or disappeared
        for i = 1:length(prev_ids)
            id = prev_ids(i);
            old_x = prev_u_int(i);
            old_y = prev_v_int(i);
            
            % Find where this star is now
            curr_idx = find(curr_ids == id);
            
            if isempty(curr_idx)
                % Star left the FOV. Brightness dropped. Trigger -1 event.
                new_events = [new_events; old_x, old_y, t_current, -1];
            else
                new_x = curr_u_int(curr_idx);
                new_y = curr_v_int(curr_idx);
                
                % Did the star cross a pixel boundary?
                if (old_x ~= new_x) || (old_y ~= new_y)
                    % Left the old pixel (-1)
                    new_events = [new_events; old_x, old_y, t_current, -1];
                    % Entered the new pixel (+1)
                    new_events = [new_events; new_x, new_y, t_current, 1];
                end
            end
        end
        
        % Check for completely new stars that just entered the FOV
        for j = 1:length(curr_ids)
            if ~ismember(curr_ids(j), prev_ids)
                % New star appeared. Brightness increased. Trigger +1 event.
                new_events = [new_events; curr_u_int(j), curr_v_int(j), t_current, 1];
            end
        end
        
    else
        % First time step: Initialize all visible stars as positive events
        for j = 1:length(curr_ids)
            new_events = [new_events; curr_u_int(j), curr_v_int(j), t_current, 1];
        end
    end
    
    % Package current integer positions to pass to the next time step
    current_state.ids = curr_ids;
    current_state.u = curr_u_int;
    current_state.v = curr_v_int;
end