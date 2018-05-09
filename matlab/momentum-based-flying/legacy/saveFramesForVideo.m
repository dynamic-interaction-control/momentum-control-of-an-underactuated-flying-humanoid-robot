function saveFramesForVideo(pathAndFileName,axVideo,aviobj,n)
    F = getframe(axVideo);
    for i = 1:n
        writeVideo(aviobj,F);
        drawnow;
    end
end