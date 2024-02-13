from itertools import product
from sys import stdout as out
from mip import Model, xsum, minimize, BINARY, INTEGER


def MIP_solver_edge_constraints(nodes,weight_matrix,in_out_dic=None):
    # number of nodes and list of vertices
    n, V = len(nodes), set(range(len(nodes)))
    print('here is V')
    print(V)
    print(n)

    # distances matrix
    c = weight_matrix
    model = Model()

    # binary variables indicating if arc (i,j) is used on the route or not
    x = [[model.add_var(var_type=BINARY) for j in V] for i in V]

    # continuous variable to prevent subtours: each city will have a
    # different sequential id in the planned route except the first one
    y = [model.add_var() for i in V]

    # objective function: minimize the distance
    model.objective = minimize(xsum(c[i][j]*x[i][j] for i in V for j in V))

    # constraint : leave each city only once
    for i in V:
        model += xsum(x[i][j] for j in V - {i}) == 1

    # constraint : enter each city only once
    for i in V:
        print(V-{i})
        print(x[j][i] for j in V - {i})
        model += xsum(x[j][i] for j in V - {i}) == 1

    # subtour elimination\
    for (i, j) in product(V - {0}, V - {0}):
        if i != j:
            model += y[i] - (n+1)*x[i][j] >= y[j]-n
    
    #add edge constraints
    # print(nodes)
    # print(len(x))

    # optimizing
    model.optimize()

    # checking if a solution was found
    solution=[]
    if model.num_solutions:
        out.write('route with total distance %g found: %s'
                % (model.objective_value, nodes[0]))
        # solution.append(nodes[0])
        nc = 0
        while True:
            nc = [i for i in V if x[nc][i].x >= 0.99][0]
            out.write(' -> %s' % nodes[nc])
            solution.append(nodes[nc])
            if nc == 0:
                break
        out.write('\n')
        print('da solution ta returnz')
        print(solution)
        return solution
    