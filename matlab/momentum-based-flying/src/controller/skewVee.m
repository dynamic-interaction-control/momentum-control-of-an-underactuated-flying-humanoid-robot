function x = skewVee(A)
%skewVee Function generates a 3X1 vector from a 3X3 A matrix. In particular,
%this function does two operations: first, it creates the skew symmetric matrix
%(A - A')/2, and then retrives from this skew symmetric matrix the 3x1
%vevtor composing it.

    S = 0.5*(A - A'); 
    x = [-S(2,3)
          S(1,3)
         -S(1,2)];

end
