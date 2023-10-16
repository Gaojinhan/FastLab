from ortools.sat.python import cp_model
import numpy as np
import csv

# The function is used to read the result of the upper problem
def readData(id,filename,R):
    goal_positions = []
    target_time = []

    with open(filename, 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        
        for row in csv_reader:
            if row['id'] == str(id):
                goal_positions.append(R[int(row['goal_positions'])])
                target_time.append(int(row['target_time']))

    array1 = np.array(goal_positions).reshape(int(len(goal_positions)/3),3)
    array2 = np.array(target_time).reshape(int(len(target_time)/3),3)

    return array1.tolist(), array2.tolist()


# The function is used to write data to a csv file
def writeDataToCSV(id,route):
    header = ['id','transporter_1', 'transporter_2', 'transporter_3', 'transporter_4', 'transporter_5', 'transporter_6']
    data = []
    for i in range(len(route[0])):
        data.append([id,route[0][i],route[1][i],route[2][i],route[3][i],route[4][i],route[5][i]])

    with open('./data/LPsolution.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        # write the header
        writer.writerow(header)
        # write the data
        writer.writerows(data)


def LpSolver(T,P,Column,Row,startPostions,R,processingTime):
    # Declare the model
    model = cp_model.CpModel()  

    # Read the schedule (the upper problem solution)
    goalPostions,targetTimes = readData(1,'./data/UPsolution.csv',R)

    numNode= Row * Column
    # Define decision variables
    x = [[[[model.NewIntVar(0,1,f'x_{i}_{j}_{t}_{k}') for k in range(P)] for t in range(T)] for j in range(numNode)] for i in range(numNode)]

    # Define intermediate variables
    y = [[model.NewIntVar(0,1,f'y_{k}_{t}') for t in range(T)] for k in range(P)]

    # N_i is the set of nodes connected to node i
    Map = np.arange(Row*Column).reshape(Row, Column)
    N = [[] for i in range(Row*Column)]

    # D_i is the set of the nodes connected to node i on the diagonal
    D = [[] for i in range(Row*Column)]

    # Four corners
    corners = [0,3,52,55]
    N[corners[0]]=[corners[0],1,4,5]
    D[corners[0]]=[corners[0],5]
    N[corners[1]]=[corners[1],2,6,7]
    D[corners[1]]=[corners[1],6]
    #N[corners[2]]=[corners[2],112,113,117]
    N[corners[2]]=[corners[2],48,49,53]
    D[corners[2]]=[corners[2],49]
    #N[corners[3]]=[corners[3],114,115,118]
    N[corners[3]]=[corners[3],50,51,54]
    D[corners[2]]=[corners[3],50]

    # Nodes in the edge
    edgeNodesB = [1,2]
    edgeNodesT = [53,54]
    edgeNodesL = []
    edgeNodesR = []
    for i in range(1,Row-1):
        edgeNodesL.append(Map[i][0])
        edgeNodesR.append(Map[i][Column-1])

    for node in edgeNodesB:
        N[node]=[node,node-1,node+1,node+Column-1,node+Column,node+Column+1]
        D[node]=[node+Column-1,node+Column+1]

    for node in edgeNodesT:
        N[node]=[node,node-1,node+1,node-Column-1,node-Column,node-Column+1]
        D[node]=[node-Column-1,node-Column+1]

    for node in edgeNodesL:
        N[node]=[node,node-Column,node-Column+1,node+1,node+Column,node+Column+1]
        D[node]=[node-Column+1,node+Column+1]

    for node in edgeNodesR:
        N[node]=[node,node-Column,node-Column-1,node-1,node+Column,node+Column-1]
        D[node]=[node-Column-1,node+Column-1]
        
    # Nodes in the center part
    edgeNodes = corners+edgeNodesB+edgeNodesT+edgeNodesL+edgeNodesR
    nodes = list(range(Row*Column))
    centerNodes = list(set(nodes)-set(edgeNodes))

    for node in centerNodes:
        N[node]=[node,node-Column-1,node-Column,node-Column+1,node-1,node+1,node+Column-1,node+Column,node+Column+1]
        D[node]=[node-Column-1,node-Column+1,node+Column-1,node+Column+1]

    # Define constraints

    # Definition of x
    for k in range(P):
        for t in range(T):
            model.Add(sum(x[i][j][t][k] for j in range(numNode) for i in range(numNode)) == 1)
            for i in range(numNode):
                model.Add(sum(x[i][j][t][k] for j in range(numNode) if j not in N[i]) == 0)
    
    for i in range(numNode):
        for t in range(T-1):
            for k in range(P):
                model.Add(sum(x[j][i][t][k] for j in range(numNode) if j in N[i]) == sum(x[i][n][t+1][k] for n in range(numNode) if n in N[i]))

    # Collision avoidance
    for j in range(numNode):
        for t in range(T):
            model.Add(sum(x[i][j][t][k] for k in range(P) for i in range(numNode) if i in N[j])<=1)

    for t in range(T):
        for i in range(numNode):
            for j in range(numNode):
                if i!=j:
                    model.Add(sum((x[i][j][t][k]+x[j][i][t][k]) for k in range(P)) <= 1)
    
    
    # Movement Constraints
    diagFlag = [[[model.NewBoolVar(f'diagFlag_{i}_{t}_{k}') for k in range(P)] for t in range(T-2)] for i in range(numNode)]
    for i in range(numNode):
        for k in range(P):
            model.Add(diagFlag[i][2][k]==sum(x[i][n][t][k] for n in range(numNode) if n in D[i]))
            model.Add(sum(x[j][j][3][k] for j in D[i])==1).OnlyEnforceIf(diagFlag[i][2][k])
            model.Add(sum(x[j][j][4][k] for j in D[i])==1).OnlyEnforceIf(diagFlag[i][2][k])
            for t in range(2,T-2):
                model.Add(diagFlag[i][t][k]<=sum(x[j][i][t-1][k] for j in range(numNode) if j in list(set(N[i])-set(D[i]))))
                model.Add(diagFlag[i][t][k]<=sum(x[i][n][t][k] for n in range(numNode) if n in D[i]))
                model.Add(diagFlag[i][t][k]>=sum(x[j][i][t-1][k] for j in range(numNode) if j in list(set(N[i])-set(D[i])))+sum(x[i][n][t][k] for n in range(numNode) if n in D[i])-1)

                model.Add(sum(x[j][j][t+1][k] for j in D[i])==1).OnlyEnforceIf(diagFlag[i][t][k])
                model.Add(sum(x[j][j][t+2][k] for j in D[i])==1).OnlyEnforceIf(diagFlag[i][t][k])

    # Start Postions 
    model.Add(x[R[5]][R[5]][0][0]==1)
    model.Add(x[R[4]][R[4]][0][1]==1)
    model.Add(x[R[1]][R[1]][0][2]==1)
    model.Add(x[R[0]][R[0]][0][3]==1)
    model.Add(x[R[3]][R[3]][0][4]==1)
    model.Add(x[R[2]][R[2]][0][5]==1)


    # Completion conditions
    for pallet in range(P):
        for task in range(len(goalPostions[0])):
            goal = goalPostions[pallet][task]
            timing = targetTimes[pallet][task]
            model.Add(sum(x[i][goal][timing][pallet] for i in range(numNode) if i in N[goal])==1)
        
            if task<2:
                for t in range(timing+1,timing+processingTime[task]):
                    model.Add(x[goal][goal][t][pallet]==1)
        
        for t in range(T):
            model.Add(sum(x[i][goal][t][pallet] for i in range(numNode) if i in N[goal])==1-y[pallet][t])
            if t<=timing-1:
                model.Add(-y[pallet][t]+y[pallet][t+1]<=0)


    # Objective Function
    cost = sum(y[k][t] for t in range(T) for k in range(P))
    model.Minimize(cost)

    solver = cp_model.CpSolver()
    status = solver.Solve(model)
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f'Maximum of objective function: {solver.ObjectiveValue()}\n')
    else:
        print('No solution found.')

    #for pallet in range(P):
    route = [[] for k in range(P)]
    for pallet in range(P):
        for i in range(numNode): 
            for j in range(numNode):
                for t in range(T):
                    if solver.Value(x[i][j][t][pallet]) == 1:
                        route[pallet].append((i,j,t,pallet))
                    
        route[pallet]=sorted(route[pallet],key= lambda item: item[2])

    writeDataToCSV(1,route)

    return route