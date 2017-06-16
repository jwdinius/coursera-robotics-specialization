function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise
flag = 1;
% *******************************************************************
for i = 1:length(P1(:,1))
    a = P1(i,:);
    if i == length(P1(:,1))
        b = P1(1,:);
        c = P1(2,:);
    else
        b = P1(i+1,:);
        if (i == 1)
            c = P1(3,:);
        else
            c = P1(1,:);
        end
    end
    A = [b(1)-a(1) c(1)-a(1);
         b(2)-a(2) c(2)-a(2)];
    resP1 = sign(det(A));
    for j = 1:length(P2(:,1))
        c = P2(j,:);
        A = [b(1)-a(1) c(1)-a(1);
             b(2)-a(2) c(2)-a(2)];
        resP2(j) = sign(det(A));
    end
    if ( (resP1 == -resP2(1)) && ...
         (resP2(1) == resP2(2)) && ...
         (resP2(2) == resP2(3)) )
        flag = 0;
        return;
    end
end
for i = 1:length(P2(:,1))
    a = P2(i,:);
    if i == length(P2(:,1))
        b = P2(1,:);
        c = P2(2,:);
    else
        b = P2(i+1,:);
        if (i == 1)
            c = P2(3,:);
        else
            c = P2(1,:);
        end
    end
    A = [b(1)-a(1) c(1)-a(1);
         b(2)-a(2) c(2)-a(2)];
    resP1 = sign(det(A));
    for j = 1:length(P1(:,1))
        c = P1(j,:);
        A = [b(1)-a(1) c(1)-a(1);
             b(2)-a(2) c(2)-a(2)];
        resP2(j) = sign(det(A));
    end
    if ( (resP1 == -resP2(1)) && ...
         (resP2(1) == resP2(2)) && ...
         (resP2(2) == resP2(3)) )
        flag = 0;
        return;
    end
end

% *******************************************************************
end