% Play/pause the animation when pressing the spacebar
function playpause(~, e)
    global isPaused;
    if strcmp(e.Key, 'space')
        isPaused = ~isPaused;
    end
end