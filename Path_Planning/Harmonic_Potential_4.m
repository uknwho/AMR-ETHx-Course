% parameter
tol      =  0.01;
maxIter  = 50;

% initialize map
Map = zeros(11,9);
Map(1,:) = -1; Map(11,:) = -1; Map(:,1) = -1; Map(:,9)     = -1;
Map(9,2) = -1; Map(10,2) = -1; Map(10,3)= -1; Map(5:6,5:8) = -1;

% initialize search start and goal locations
SearchStart = [3,7];
SearchGoal  = [9,6];

% initialize iterative search
SearchSolution = zeros(size(Map));
SearchSolution(Map==-1)=1;   %set obstacle cells to "1"
SearchSolution(Map==0) =0.5; %set free cells to "0.5"
SearchSolution(SearchGoal(1),SearchGoal(2)) = 0;
        
% iteratively solve the discrete Laplace Equation with Dirichlet boundary conditions
iter = 0; maxChange = inf;
while maxChange > tol
    iter = iter+1;
    assert(maxIter > iter, 'maxIter assert triggered. Aborting.');

    NextSearchSolution = SearchSolution;
    for x=1:1:size(Map,1)
        for y=1:1:size(Map,2)
            if and(SearchSolution(x,y)~=0,SearchSolution(x,y)~=1)
                NextSearchSolution(x,y) = 1/4*(SearchSolution(x-1,y) + ...
                                               SearchSolution(x+1,y) + ...
                                               SearchSolution(x,y-1) + ...
                                               SearchSolution(x,y+1) );
            end
        end
    end
        
    maxChange = max(max(abs(SearchSolution-NextSearchSolution)));
    SearchSolution = NextSearchSolution;
end
        
% extract solution path from start to goal
i=SearchStart(1);
j=SearchStart(2);
k=SearchGoal(1);
l=SearchGoal(2);

c=2;
OptimalPath(1,1)=i;
OptimalPath(1,2)=j;

while ~isequal([i,j],[k,l])
    u=[SearchSolution(i-1,j) SearchSolution(i+1,j) SearchSolution(i,j-1) SearchSolution(i,j+1) SearchSolution(i+1,j+1) SearchSolution(i-1,j+1) SearchSolution(i+1,j-1) SearchSolution(i-1,j-1)];
    [Nextstep, post]=min(u);
    if post==1
        OptimalPath(c,1)=i-1;
        OptimalPathe(c,2)=j;
    end
    if post==2
        OptimalPath(c,1)=i+1;
        OptimalPath(c,2)=j;
    end
    if post==3
        OptimalPath(c,1)=i;
        OptimalPath(c,2)=j-1;
    end
    if post==4
        OptimalPath(c,1)=i;
        OptimalPath(c,2)=j+1;
    end
    if post==5
        OptimalPath(c,1)=i+1;
        OptimalPath(c,2)=j+1;
    end
    if post==6
        OptimalPath(c,1)=i-1;
        OptimalPath(c,2)=j+1;
    end
    if post==7
        OptimalPath(c,1)=i+1;
        OptimalPath(c,2)=j-1;
    end
    if post==8
        OptimalPath(c,1)=i-1;
        OptimalPath(c,2)=j-1;
    end
    i=OptimalPath(c,1);
    j=OptimalPath(c,2);
    c=c+1;
    
end
disp(OptimalPath);
        
% display the solution
% disp(OptimalPath)