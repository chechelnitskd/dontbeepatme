import MapGraph
import MapGraph2
import MacState
import SearchSolver

# Alternate map of spaces on 2nd floor of OLRI (not used right now)
# newOlin = MapGraph.readMapFile("olinGraph.txt")
# print("Read in Olin graph")

# Reading in the Macalester campus graph
#macGraph = MapGraph.readMapFile("macGraph.txt")
grovelandGraph = MapGraph.readMapFile("grovelandMap.txt")
grovelandHeartGraph = MapGraph2.readMapFile("grovelandHeartMap.txt")

cont = 0
while(cont != 1):
    var = int(input("Enter what kind of search: 0 for least stressful, 1 for fastest?"))
    print(var)
    if(var == 0):
        x = int(input("Enter start node:"))
        y = int(input("Enter end node:"))
        var1 = MacState.AStarMacAdvisor(grovelandHeartGraph, x, y)
        var2 = SearchSolver.BestFirstSearchSolver(var1)
        var2.initSearch()
        while(True):
            result = var2.searchStep()
            print(result[0])
            if(result[2] == "Done"):
                break
    else:
        x2 = int(input("Enter start node:"))
        y2 = int(input("Enter end node:"))
        var3 = MacState.AStarMacAdvisor(grovelandGraph, x2, y2)
        var4 = SearchSolver.BestFirstSearchSolver(var3)
        var4.initSearch()
        while(True):
            result = var4.searchStep()
            print(result[0])
            if(result[2] == "Done"):
                break
    cont = int(input("Continue? (0 for yes, 1 for no)"))

#print(MacState.MazeTaskAdvisor(MapGraph))

# Examples of how to use the MapGraph:

# Find the neighbors of node 38, the junction between the path in front of Olin-Rice and the path to the rock garden
# neighInfo = macGraph.getNeighbors(38)
# print("Neighbors of node", 38, "and the cost to each neighbor")
# print("Neighbor   Cost")
# for neigh, cost in neighInfo:
#     print(neigh, '      ', round(cost, 3))

# # Find the heuristic, straight-line distance between node 38 and node 94, the Campus Center
# hCost = macGraph.heuristicDist(38, 94)
# print("Neighbors of node 85:")
# print(macGraph.getNeighbors(85))
# print("Straight-line distance between 85 and 86:", macGraph.heuristicDist(85, 86))
# print("Straight-line distance between 85 and 12:", macGraph.heuristicDist(85, 12))





