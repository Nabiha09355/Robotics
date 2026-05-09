

function stopPipelineSafely(pipe)
    try
        pipe.stop();
    catch
        % Suppress errors if the pipe was already stopped
    end
end