%%SYMS tutorial script

%Start by declaring your symbolic variables with the 'syms' command
syms x_ y_ ;

%Define your system of equations, being sure to use '==' in the equation,
%and '=' as the assignment of the equation to a variable name
eqn1 = 2*x_ +5*y_ == 27;
eqn2 = 3*x_ - 2*y_ == 6;

%For convenience, make a vector of equations
eqns=[eqn1,eqn2];

%Use the solve funtion
S = solve(eqns);


%Be sure to convert the syms variables back into a usable format using the
%'double()' function
x = double(S.x_(1,1))
y = double(S.y_(1,1))

