function part = fromIndexToPart(index)
    switch index
        case 1
            part = '\emph{Torso}';
        case 2
            part = '\emph{Left arm}';
        case 3
            part = '\emph{Right arm}';
        case 4
            part = '\emph{Left leg}';
        case 5
            part = '\emph{Right leg}';
        otherwise
            part = '\emph{Ill-posed index}';
    end 
end