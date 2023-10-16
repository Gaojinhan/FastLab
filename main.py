import UpSolverCPmodel, LpSolver

def main():
    # Define System Variables

    # Assume that there are 4 robots and 5 products need to be processed.
    # Each product process contains 3 tasks.
    R=6
    P=6
    T=3

    # Estimated Processing Time
    E = [50,28,48]

    # Time of changing pen
    changeTime = 40

    # Assume that 6 robots are in use.
    Robots = [0,28,52,3,31,55]

    # Define the the number of gird using for this simulation
    Row = 30
    Column = 4

    # Color Setting
    color = [[1,1,2],[2,3,1],[3,2,2],[1,3,2],[3,1,3],
            [3,1,2]]

    # Solve the upper problem and collect data
    startPositions,startTime=UpSolverCPmodel.CPmodelSolver(R,P,T,E,changeTime,color,Robots)
    # Solve the 
    LpSolver.LpSolver(T,P,Column,Row,startPositions,R,startTime)
    pass

if __name__ == "__main__":
    main()