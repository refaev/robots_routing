# robots_routing

Solve the Multi-Agent Path Finding (MAPF) problem for two robots assuming grid-world.

## Usage: ## 
python manager.py

## Design: ##
![Project design](https://refaev.github.com/robots_routing//images/design.png)

## Related work: ##

From [1]:
Solving the Multi-Agent Path Finding (MAPF) problem optimally is known to be NP-Hard for both make-span and total arrival time minimization. 
Many types of optimal MAPF algorithms and their variants have been proposed, including Conflict Based Search (CBS) [14], a method based on branch-and-cut-and-price (BCP) [10], and a boolean satisfiability based algorithm (SAT) [19].

From [15]: 
Reduction-based solversThis class of solvers, used in recent work, reduces MAPF to other problems that are well studied in computer science. Prominent examples include reducing to Boolean Satisfiability (SAT)[50], Integer Linear Programming (ILP)[55]and Answer Set Programming (ASP)[13].
Search-based suboptimal solversSearch-based solvers usually aim to provide a high quality solution (close to optimal) but they are not complete for many cases. These solvers differ by the way they treat conflicts between agents. A prominent example of a search-based sub-optimal algorithm is Hierarchical Cooperative A*(HCA*)[43]. In HCA* the agents are planned one at a time according to some predefined order. Once the first agent finds a path to its goal, that path is written (reserved) into a global reser-vation table.
Rule-based approaches include specific movement rules for different scenarios and usually do not include massive search. The agents plan their route according to the specific rules. Rule-based solvers favor completeness at low computational cost over solution quality.TASS[25]and Push and Swap (and its variants)[30,37,10]are two recently proposed rule-based MAPF sub-optimal algorithms that run in polynomial time
Some suboptimal solvers are hybrids and include specific movement rules as well as significant search. For example, if the graph is a grid then establishing flow restrictions similar to traffic laws can simplify the problem.
Optimal MAPF solvers usually search a global search space which combines the individual states of all kagents. This state space is denoted as the k-agent state space. The states in the k-agent state space are the different ways to place kagents into |V|vertices, one agent per vertex. In the start and goal states agent aiis located at vertices startiand goali, respectively. Operators between states are all the non-conflicting actions (including wait) that all agents have. Given this general state space, any A*-based algorithm can be used to solve the MAPF problem optimally.


From [14]:
CBS has two levels. The low-level finds optimal paths for the individual agents. If the paths conflict, the high level, via a split action, imposes constraints on the conflicting agents to avoid these conflicts.”

Our chosen approach is similar to HCA: The first robot plans its path using all obstacles. Then this path is embded as an abstacle in a 3D map, with time as the third dimension. 
The second robot uses this 3D map to find shotest path.

[1] https://arxiv.org/abs/2102.12461:
[10] Edward Lam, Pierre Le Bodic, Daniel Damir Harabor, and Peter J Stuckey. 2019. Branch-and-Cut-and-Price for Multi-Agent Pathfinding. In Proc. of the Intl Joint Conf on Artificial Intelligence. 1289–1296.
[14] Guni Sharon, Roni Stern, Ariel Felner, and Nathan R Sturtevant. 2015. ConflictBased Search for Optimal Multi-Agent Pathfinding. Artificial Intelligence 219 (2015), 40–66
[15]https://reader.elsevier.com/reader/sd/pii/S0004370214001386?token=19B8F979A72223D91890F02BEE8377FC1B934E7D9922C7E898E0E20D43BF8495740AF7F075D4CED9B574F28F9C40EE62&originRegion=eu-west-1&originCreation=20210506135614
[19] Pavel Surynek, Ariel Felner, Roni Stern, and Eli Boyarski. 2016. Efficient SAT Approach to Multi-Agent Path Finding Under the Sum of Costs Objective. In Proc. of the European Conf on Artificial Intelligence. 810–818.

