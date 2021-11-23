#include <iostream>
#include <ilcplex/ilocplex.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <math.h>
using namespace std;

int main()
{
    // Create data
        // Suppose that there are 15 demand nodes, and root node is located at node 0
    int numDemandNodes = 15, rootNode = 0, numNodes = numDemandNodes+1;
        // Suppose that there are 5 trucks, and each has capcity of 25
    int numTrucks = 5, capacity = 25;

    srand(time(NULL));
    int xPos[numNodes], yPos[numNodes];
    for(int i=0; i<numNodes; i++) {
        xPos[i] = rand()%100;
        yPos[i] = rand()%100;
    }

    vector<vector<double>> cost(numNodes, vector<double>(numNodes));
    for(int i=0; i<numNodes; i++) {
        for(int j=0; j<numNodes; j++) {
            cost[i][j] = sqrt(pow(xPos[j] - xPos[i], 2) + pow(yPos[j] - yPos[i], 2));
        }
    }

    // demand of nodes
    vector<double> d(numNodes);
    for(int i=0; i<numNodes; i++) {
        if(i != rootNode) {
            d[i] = 1 + rand()%10;
        }
    }
    IloEnv env;

    try{
        IloModel model(env);


        /*------DECISION VARIABLE--------*/
        IloArray<IloNumVarArray> x(env, numNodes);
        for(int i=0; i<numNodes; i++) {
            x[i] = IloNumVarArray(env, numNodes);
            for(int j=0; j<numNodes; j++) {
                x[i][j] = IloNumVar(env, 0, 1, ILOBOOL);
            }
        }


        /*---------OBJECTIVE FUNTION--------*/
        IloExpr obj(env);
        for(int i=0; i<numNodes; i++) {
            for(int j=0; j<numNodes; j++) {
                obj += x[i][j]*cost[i][j];
            }
        }
        model.add(IloMinimize(env, obj));
        obj.end();


        /*----------CONSTRAINTS-------------*/
        IloExpr expr(env), expr2(env);
        for(int i=0; i<numNodes; i++) {
            model.add(x[i][i] == 0);
        }

        for(int i=0; i<numNodes; i++) {
            if(i != rootNode) {
                for(int j=0; j<numNodes; j++) {
                    expr += x[i][j];
                    expr2 += x[j][i];

                }
                model.add(expr == 1);
                model.add(expr2 == 1);
                expr.clear();
                expr2.clear();
            }
        }

        for(int i=0; i<numNodes; i++) {
            expr += x[rootNode][i];
            expr2 += x[i][rootNode];
        }
        model.add(expr == numTrucks);
        model.add(expr2 == numTrucks);
        expr.clear();
        expr2.clear();

        // f[i][j] denotes the load on the truck from i to j


        IloArray<IloNumVarArray> f(env, numNodes);
        for(int i=0; i<numNodes; i++) {
            f[i] = IloNumVarArray(env, numNodes);
            for(int j=0; j<numNodes; j++) {
                f[i][j] = IloNumVar(env, 0, capacity, ILOINT);
            }
        }

        for(int i=0; i<numNodes; i++) {
            if(i != rootNode)  {
                for(int j=0; j<numNodes; j++) {
                    expr += f[i][j];
                    expr2 += f[j][i];
                }
                model.add(expr2-expr == d[i]);
                expr.clear();
                expr2.clear();
            }
        }

        for(int i=0; i<numNodes; i++) {
            for(int j=0; j<numNodes; j++) {
                model.add(f[i][j] <= capacity*x[i][j]);
            }
        }


        // solve and print solution
        IloCplex cplex(model);
        cplex.solve();

        double minSumCost = cplex.getObjValue();
        cout<< "The minimum cost is "<< minSumCost<< "\n";
        cout<< "The path is:"<< "\n";
        for(int i=0; i<numNodes; i++) {
            if(cplex.getValue(x[rootNode][i]) == 1) {
                cout<< rootNode<< "->"<< i;
                int j = i;
                while(j != rootNode) {
                    for(int k=0; k<numNodes; k++) {
                        if(cplex.getValue(x[j][k]) == 1) {
                            cout<< "->"<< k;
                            j = k;
                            break;
                        }
                    }
                }
                cout<< "\n";
            }
        }

        expr.end();
        expr2.end();


    }


    catch (IloException& e){
        cerr << "Conver exception caught: " << e << endl; // No solution exists
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }

    env.end();


    return 0;
}











