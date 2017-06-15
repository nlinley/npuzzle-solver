import resource

#required for argument handling
import sys
import heapq
import math
import time

starttime = time.time()

class node_boardstate:
    def __init__(self,parent,moveDirection,positionList):
        self.unique_node_id = int("".join(positionList))
        self.moveDirection = moveDirection
        self.parent = parent
        self.positionList = positionList
        self.heuristicCost = 0
        self.Ida_F = 0
        #count current number of moves to get to this node
        if parent:
            self.costToGetHere = parent.costToGetHere + 1
        else:
            self.costToGetHere = 0

    def fnct_possibleMoves(self):
        #create children  in Up, down, left, right order
        zeroPos = self.positionList.index('0')
        possibleMoveList = []
        if zeroPos > (n_dimensions - 1):
            #can go up
            possibleMoveList.append( self.create_childNode(zeroPos, zeroPos - 3, "Up"))
        if zeroPos < (n_dimensions * (n_dimensions -1)):
            #can go down
            possibleMoveList.append(  self.create_childNode(zeroPos, zeroPos + 3, "Down"))
        if (zeroPos %  n_dimensions) > 0:
            #can go left
            possibleMoveList.append( self.create_childNode(zeroPos, zeroPos - 1, "Left"))
        if (zeroPos % n_dimensions) < 2:
            #can go right
            possibleMoveList.append( self.create_childNode(zeroPos, zeroPos + 1, "Right"))

        return possibleMoveList


    def create_childNode(self,zeroPosition,swapPosition,Direction):
        #create new node_boardstate object with list items swapped and direction mentioned

        newPositionList = self.positionList.copy()
        newPositionList[zeroPosition], newPositionList[swapPosition] = newPositionList[swapPosition], newPositionList[zeroPosition]
        return node_boardstate(self,Direction,newPositionList)

    def is_goal(self):
        if self.unique_node_id == GBL_goal_state:
            return True
        else:
            return False

    def heuristic_Manhattan(self):
        total_distance = 0
        #create 2 lists of lists for the final goal view and the current state
       # goal_3D = [[0,1,2],[3,4,5],[6,7,8]]
        goal_3D = []
        curValue = 0
        for i in range(0, n_dimensions):
            mySlice=[]
            for j in range(0, n_dimensions):
                mySlice.append(curValue)
                curValue += 1
            goal_3D.append(mySlice)
         #need to fix this   
        myCurPosition = []
        for i in range(0, n_dimensions):
            mySlice = list(map(int, self.positionList[(n_dimensions * i):(n_dimensions * (i+1))]))
            myCurPosition.append(mySlice)

        for i in range(n_dimensions**2):
            xpos = -1
            ypos = -1
            expect_xpos = -1
            expect_ypos = -1
            for j in range(n_dimensions):
                if i in myCurPosition[j]:
                    xpos = myCurPosition[j].index(i)
                    ypos = j
                    break
            #at this point we have an X and Y position for the number we are looking for.  We can compare these to where they are expected
            myPieceCost = 0
            for j in range(n_dimensions):
                if i in goal_3D[j]:
                    expect_xpos = goal_3D[j].index(i)
                    expect_ypos = j
                    break
            #this sets our expected positions
            myPieceCost += abs(xpos - expect_xpos)
            myPieceCost += abs(ypos - expect_ypos)
            total_distance += myPieceCost

        #help preserve the order Up, down, left, right order by adjusting the priority just a bit to differentiate
        #between same cost nodes
        if self.moveDirection == "Up":
            total_distance += 0.1
        elif self.moveDirection == "Down":
            total_distance += 0.2
        elif self.moveDirection == "left":
            total_distance += 0.3
        elif self.moveDirection == "Right":
            total_distance += 0.4
        #total_distance = int(total_distance)
        self.heuristicCost = total_distance
        return total_distance


class basic_stack:
    def __init__(self):
        self.stackitems = []

    def push(self,stackitem):
        self.stackitems.append(stackitem)

    def pop(self):
        return self.stackitems.pop()

    def isEmpty(self):
        #if self.stackitems.count == 0:
        if len(self.stackitems) == 0:
            return True
        else:
            return False
    def size(self):
        return len(self.stackitems)

    def contains(self,someObject):
        if someObject in self.stackitems:
            return True
        else:
            return False

class priority_queue:
        #slighty modified use of heapq
    def __init__(self):
        #use a set to contain reference items for what is there
        #heap of cost + uniquenodeID number + objectID of node
        self.heap = []
        heapq.heapify(self.heap)

    def push(self,node):
        cost = node.costToGetHere + node.heuristic_Manhattan()
        heapq.heappush(self.heap, (cost,node.unique_node_id,node))

    def pop(self):
        try:
            return heapq.heappop(self.heap)
        except IndexError:
            return None

    def check_for_node(self,uniqueID):
        #provide capability to check for existence of a given board state in the heap
        for item in self.heap:
            if item[1] == uniqueID:
                return True
        return False

    def isEmpty(self):
        if len(self.heap) == 0:
            return True
        else:
            return False

    def size(self):
        return len(self.heap)

    def decreasekey(self,node):
        for item in self.heap:
            if item[1] == node.unique_node_id:
                if item[0] > node.costToGetHere:
                    #if old entry cost is higher than the current node, swap them out
                    self.heap.remove(item)
                    self.push(node)


def getSolution(goalNode):
    global GBL_goal_depth
    global GBL_max_depth
    count_Steps_To_Goal = 0 #need to backtrace the parents and get move direction
    reversedlist = [goalNode.moveDirection]
    traverse_parent = goalNode.parent
    while traverse_parent:
        #step backwards through the parent objects of our selected goal, increment total steps and put steps into a list
        count_Steps_To_Goal = count_Steps_To_Goal + 1
        try:
            reversedlist.append(traverse_parent.moveDirection)
        except:
            pass
        traverse_parent = traverse_parent.parent
    reversedlist.reverse()
    reversedlist = reversedlist[1:]
    GBL_goal_depth = count_Steps_To_Goal
    if GBL_max_depth == 0: GBL_max_depth = GBL_goal_depth
    
    print("path_to_goal:", reversedlist)
    print("cost_of_path:", GBL_goal_depth)
    print("nodes_expanded:", GBL_nodes_expanded)
    print("fringe_size:", GBL_current_frontier_size)
    print("max_fringe_size:", GBL_max_frontier_size)
    print("search_depth:", GBL_goal_depth)
    print("max_search_depth:", GBL_max_depth)
    print("running_time:", time.time() - starttime)
    print("max_ram_usage:", resource.getrusage(resource.RUSAGE_SELF).ru_maxrss /1024)
  
    print("path_to_goal:", reversedlist,  file=outfile)
    print("cost_of_path:", GBL_goal_depth,  file=outfile)
    print("nodes_expanded:", GBL_nodes_expanded,  file=outfile)
    print("fringe_size:", GBL_current_frontier_size,  file=outfile)
    print("max_fringe_size:", GBL_max_frontier_size,  file=outfile)
    print("search_depth:", GBL_goal_depth,  file=outfile)
    print("max_search_depth:", GBL_max_depth,  file=outfile)
    print("running_time:", time.time() - starttime,  file=outfile)
    print("max_ram_usage:", resource.getrusage(resource.RUSAGE_SELF).ru_maxrss /1024,  file=outfile) 
    outfile.close()

   
def search_BFS():
    global GBL_max_frontier_size
    global GBL_current_frontier_size
    import queue
    #create 2 queues, one for the objects and one for the ID# of the an object.  Presumably we could have different
    #objects that have the same board state and we wouldn't be able to tell that in the object queue
    the_frontier = queue.Queue()
    the_frontier_nodeID = queue.Queue()
    explored_nodeIDs = set()
    startNode = node_boardstate(None,None,GBL_initialBoardState)

    the_frontier.put(startNode)
    the_frontier_nodeID.put(startNode.unique_node_id)
    explored_nodeIDs.add(startNode.unique_node_id)

    #update queue size
    if the_frontier.qsize() > GBL_max_frontier_size:
        GBL_max_frontier_size = the_frontier.qsize()

    global GBL_nodes_expanded
    global GBL_max_depth
    
    while not the_frontier.empty():

        curstate = the_frontier.get()
        throwaway = the_frontier_nodeID.get()
        explored_nodeIDs.add(curstate.unique_node_id)
        GBL_nodes_expanded = GBL_nodes_expanded + 1

        if curstate.costToGetHere > GBL_max_depth:
            GBL_max_depth = curstate.costToGetHere
        if curstate.is_goal():
            GBL_nodes_expanded = GBL_nodes_expanded - 1
            GBL_max_depth += 1
            GBL_current_frontier_size = the_frontier.qsize()
            getSolution(curstate)
            return True

        for search_step in curstate.fnct_possibleMoves():

            if not ((search_step.unique_node_id in explored_nodeIDs) or (search_step.unique_node_id in the_frontier_nodeID.queue)):
                the_frontier.put(search_step)
                the_frontier_nodeID.put(search_step.unique_node_id)

        if the_frontier.qsize() > GBL_max_frontier_size:
            GBL_max_frontier_size = the_frontier.qsize()
        
        

    #if we get this far, we didn't find a solution
    return False

def search_DFS():
    global GBL_max_depth
    global GBL_max_frontier_size
    global GBL_current_frontier_size
    global GBL_nodes_expanded

    #2 stacks, one for objects to process and one for ID's to help check existence in frontier
    dfs_frontier = basic_stack()
    dfs_frontier_nodeIDs = basic_stack()
    explored_nodeIDs = set()

    startNode = node_boardstate(None,None,GBL_initialBoardState)

    dfs_frontier.push(startNode)
    dfs_frontier_nodeIDs.push(startNode.unique_node_id)
    while not dfs_frontier.isEmpty():
        try:
            curstate = dfs_frontier.pop()
            throwaway = dfs_frontier_nodeIDs.pop()
            GBL_nodes_expanded = GBL_nodes_expanded + 1
        except IndexError:
            pass

        #update global max depth counter if required
        if curstate.costToGetHere > GBL_max_depth:
            GBL_max_depth = curstate.costToGetHere


        explored_nodeIDs.add(curstate.unique_node_id)

        if curstate.is_goal():
            GBL_nodes_expanded = GBL_nodes_expanded - 1
            GBL_current_frontier_size = dfs_frontier.size()
            getSolution(curstate)
            return True

        #presume that we need to reverse this as it is a stack and we're following specific move direction orders
        possibleMoves =  curstate.fnct_possibleMoves()
        possibleMoves.reverse()
        for search_step in possibleMoves:
            if not ((search_step.unique_node_id in explored_nodeIDs) or ( dfs_frontier_nodeIDs.contains(search_step.unique_node_id))):
                dfs_frontier.push(search_step)
                dfs_frontier_nodeIDs.push(search_step.unique_node_id)

        if dfs_frontier.size() > GBL_max_frontier_size:
            GBL_max_frontier_size = dfs_frontier.size()


    #if we get here, no result was found
    return False

def search_Astar():
    #heuristic calculations are offloaded into the priority queue class above
    global GBL_max_depth
    global GBL_max_frontier_size
    global GBL_current_frontier_size
    global GBL_nodes_expanded

    explored_nodeIDs = set()
    the_frontier = priority_queue()
    startNode = node_boardstate(None,None,GBL_initialBoardState)
    the_frontier.push(startNode)

    while not the_frontier.isEmpty():
        curstateTuple = the_frontier.pop()
        curstate = curstateTuple[2]
        explored_nodeIDs.add(curstate.unique_node_id)
        GBL_nodes_expanded = GBL_nodes_expanded + 1

        if curstate.is_goal():
            GBL_current_frontier_size = the_frontier.size()
            GBL_nodes_expanded = GBL_nodes_expanded - 1
            getSolution(curstate)
            return True

        for search_step in curstate.fnct_possibleMoves():

            is_node_in_frontier = the_frontier.check_for_node(search_step.unique_node_id)

            if not ((search_step.unique_node_id in explored_nodeIDs) or is_node_in_frontier ):
                the_frontier.push(search_step)
            elif is_node_in_frontier:
                the_frontier.decreasekey(search_step)

            if search_step.costToGetHere > GBL_max_depth:
                GBL_max_depth = search_step.costToGetHere

        if the_frontier.size() > GBL_max_frontier_size:
            GBL_max_frontier_size = the_frontier.size()

    return False

def search_IDAstar():
    global GBL_max_depth
    global GBL_max_frontier_size
    global GBL_current_frontier_size
    global GBL_nodes_expanded
    global bestcost

    startNode = node_boardstate(None,None,GBL_initialBoardState)
    h = startNode.heuristic_Manhattan()
    #bestcost = h * n_dimensions
    bestcost = h 
    print("bestcost",bestcost) 
    maxDepth = int( n_dimensions**4/2 ) #rough calculation to give enough room based on solution sizes given on wikipedia for various dimensions
    
    for curDepth in range(1, maxDepth):
        GBL_nodes_expanded = 0
         #2 stacks, one for objects to process and one for ID's to help check existence in frontier
        dfs_frontier = basic_stack()
        dfs_frontier_nodeIDs = basic_stack()
        explored_nodeIDs = set()
        dfs_frontier.push(startNode)
        
        dfs_frontier_nodeIDs.push(startNode.unique_node_id)
        while not dfs_frontier.isEmpty():
            try:
                curstate = dfs_frontier.pop()
                throwaway = dfs_frontier_nodeIDs.pop()
                GBL_nodes_expanded = GBL_nodes_expanded + 1
            except IndexError:
                pass

            #update global max depth counter if required
            if curstate.costToGetHere > GBL_max_depth:
                GBL_max_depth = curstate.costToGetHere

             
            explored_nodeIDs.add(curstate.unique_node_id)

            if curstate.is_goal():
                GBL_nodes_expanded = GBL_nodes_expanded - 1
                GBL_current_frontier_size = dfs_frontier.size()
                print(curstate.heuristic_Manhattan())
                getSolution(curstate)
                return True

            #presume that we need to reverse this as it is a stack and we're following specific move direction orders
            possibleMoves =  curstate.fnct_possibleMoves()
            possibleMoves.reverse()
            for search_step in possibleMoves:
                
                if (not ((search_step.unique_node_id in explored_nodeIDs) or 
                    ( dfs_frontier_nodeIDs.contains(search_step.unique_node_id)))) and search_step.costToGetHere != curDepth :
                    f = search_step.costToGetHere + search_step.heuristic_Manhattan()
                    #print (f, h +  n_dimensions * 2 )
                  
                    if int(f) < maxDepth and int(f) < (bestcost + search_step.costToGetHere ):
                        #bestcost = f +  n_dimensions **2
                        dfs_frontier.push(search_step)
                        dfs_frontier_nodeIDs.push(search_step.unique_node_id)
                        print("pushed",search_step.heuristicCost)
                        
                    #else:
                       # print ("pruned: ",  search_step.unique_node_id,  f,  bestcost)
            if dfs_frontier.size() > GBL_max_frontier_size:
                GBL_max_frontier_size = dfs_frontier.size()


    #if we get here, no result was found
    return False



#process arguments provided to program.  argv[1] is the algorithm to use, the other is a csv list of positions for initial state
GBL_algoToUse = sys.argv[1]
GBL_initialBoardState = list(sys.argv[2].split(','))

#set some global counters for final output

n_dimensions = int(math.sqrt(len(GBL_initialBoardState)))
GBL_goal_state = GBL_initialBoardState.copy()
GBL_goal_state.remove('0')
GBL_goal_state = list(map(int, GBL_goal_state))
GBL_goal_state.sort()
GBL_goal_state = list(map(str, GBL_goal_state))
GBL_goal_state = int("".join(GBL_goal_state))

GBL_nodes_expanded = 0
GBL_goal_depth = 0
GBL_max_frontier_size = 0
GBL_max_depth = 0
GBL_current_frontier_size = 0
GoalNode = None
outfile = open('output.txt','w')

#print ("".join(GBL_initialBoardState))
#print (int("".join(GBL_initialBoardState)))

if GBL_algoToUse == "dfs":
    search_DFS()
elif GBL_algoToUse == "bfs":
    search_BFS()
elif GBL_algoToUse == "ast":
    search_Astar()
elif GBL_algoToUse == "ida":
    search_IDAstar()

    


##OUTFILE SAMPLE
#path_to_goal: ['Up', 'Left', 'Left']
#cost_of_path: 3
#nodes_expanded: 10
#fringe_size: 11
#max_fringe_size: 12
#search_depth: 3
#max_search_depth: 4
#running_time: 0.00188088
#max_ram_usage: 0.07812500



