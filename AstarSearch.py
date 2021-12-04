class AstarSearch:

    #creat class constructor that takes two parameters: tree and heuristic 
    def __init__(self, tree:dict, heuristic:dict):
        self.tree       = tree
        self.heuristic  = heuristic 

    #A* search algorithm function
    #this function used constructor variables and returns two parameters : visited node & Optimal path
    def a_star_search_algorithm(self, start:chr, Goal:chr):
        self.expand_node = []     # Expanded nodes
        self.fringe_nodes = []     # Fringe nodes
        self.path_cost = {start: 0}     #initial cost of paths
        self.start_heuristic = self.heuristic[start]       #getting start node heuristic
        self.fringe_nodes = [[start,self.start_heuristic]] #add start state and its heuristic to fringe

        #find expanded nodes
        while True:
            self.fn_values = [i[1] for i in self.fringe_nodes] # take f(n) value for all nodes in fringe
            self.small_fn_index = self.fn_values.index(min(self.fn_values)) # take index of smallest f(n) from fn_values
            self.small_node_fn_char  = self.fringe_nodes[self.small_fn_index][0]  # take character that related to index

            self.expand_node.append(self.fringe_nodes[self.small_fn_index]) # add smallest f(n) node to expand
            del self.fringe_nodes[self.small_fn_index] # delete smallest node value from fringe
            if self.expand_node[-1][0] == Goal:  # break the loop if Goal state has been expanded
                break
            
            for self.item in self.tree[self.small_node_fn_char]: # go throug all current state neighbores
                if self.item[0] in [self.expand_item[0] for self.expand_item in self.expand_node]: # check if any of current node neighbors in expand node then skip it
                    continue
                             # current node char:   path cost for previous node              + actual cost to current node 
                self.path_cost.update({self.item[0]: self.path_cost[self.small_node_fn_char] + self.item[1]}) # add the total actual cost from the start node to the current node

                                #path cost for previous node            + current node heuristic  + actual cost to current node 
                self.fn_value = self.path_cost[self.small_node_fn_char] + heuristic[self.item[0]] + self.item[1] # calculate f(n) value for current node
                self.temp = [self.item[0], self.fn_value] # take currnt node char and its f(n) value 
                self.fringe_nodes.append(self.temp) # store temp in fringe

        return self.fringe_nodes, self.expand_node, self.path_cost.get(Goal)

   #this function recieve the goal state and parint the optimal path  
    def optimal(self, Goal):
        self.optimal_path = [Goal] # optimal node sequence
        for i in range(len(self.expand_node)-2,-1,-1): # start from last node in expand and move until reach the first node
            self.current_node_char = self.expand_node[i][0] # take current node char 
            if Goal in [self.children[0] for self.children in self.tree[self.current_node_char]]: # check if current node related to goal
                self.optimal_path.append(self.current_node_char) # add current node to optimal path
                Goal = self.current_node_char # change goal to current node
        self.optimal_path.reverse()
        return self.optimal_path

  


if __name__ == "__main__":
    tree = {
        'A': [ ['S1',1] ],
        'B': [ ['S1',1] ],
        'C': [ ['S1',1] ],
        'D': [ ['S1',1] ],
        'S1': [ ['R1',3], ['R3',5] ],
        'R1': [ ['R3',1], ['R2',4], ['R4',6] ],
        'R2': [ ['S2',6] ],
        'R3': [ ['R1',1], ['R4',5] ],
        'R4': [ ['S2',4] ],
        'S2': [ ['G1', 1], ['G2', 1], ['G3', 1], ['G4', 1] ],
        'G1': [ ['S2',1] ],
        'G2': [ ['S2',1] ],
        'G3': [ ['S2',1] ],
        'G4': [ ['S2',1] ]
        
    }

    heuristic = {'A': 14, 'B': 14, 'C': 14, 'D': 14, 'S1': 13, 'R1': 10,'R2': 7, 'R3': 9, 'R4': 6, 'S2': 1, 'G1': 0, 'G2': 0, 'G3': 0, 'G4': 0}

    try:
        start = input("Please Enter Start Noode: ")
        Goal = input("Please Enter Goal Node: ")
        astar= AstarSearch(tree, heuristic)
        fringe, expand, cost = astar.a_star_search_algorithm(start.upper(), Goal.upper())
        optimal = astar.optimal(Goal.upper())
        print("Fringe Nodes: ",fringe)
        print("Expand Nodes: ", expand)
        print("Optimal Path: ", optimal)
        print("Total Cost: ", cost)
    except :
        print("\n Wrong Chooise!, Please Try Again with Valid Input.")
