function is_use = isUsableInputPoints(is_close, is_similar, is_first)

    %==== Determines if each input point should be added into the fusion map based on the three boolean matrices ====
    is_use = (is_close & is_similar) | is_first;

end