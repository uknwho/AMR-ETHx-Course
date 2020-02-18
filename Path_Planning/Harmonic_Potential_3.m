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
disp(SearchSolution)

% iteratively solve the discrete Laplace Equation with Dirichlet boundary condition
iter=1;
intial=inf(size(SearchSolution)); %% to get a matrix of highest number for the loop.
while iter<maxIter && max(abs(intial(:) - SearchSolution(:))) > tol
    intial= SearchSolution;
    [x,y]=size(intial);
    for i=1:x
        for j=1:y
            if intial(i,j)==1 || intial(i,j)==0 % when obstacle or goal is reached.
                SearchSolution(i,j)=intial(i,j);
            end
            if intial(i,j)~=1 && intial(i,j)~=0 
                SearchSolution(i,j)=(intial(i+1,j)+intial(i-1,j)+intial(i,j+1)+intial(i,j-1))/4;
            end
        end
    end
    iter=iter+1;
end
    
    
    
disp(SearchSolution)
            