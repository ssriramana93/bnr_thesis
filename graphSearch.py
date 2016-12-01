import numpy as np 
import scipy as sp 
import Queue as Q
from copy import deepcopy

def cov(map,edge,value):
	distance = map.get(edge[0]).get(edge[1])
	return(value + (value + 1)*distance/10)

def processBranch(map,branch,cost,child):
	node = branch[-1]
	total_visit_allowed = 2

	edge = (node.name,child)
	distance = node.distance + map.get(edge[0]).get(edge[1])

	count = findVisitCount(child,branch)
	if (count > 1 and count <= total_visit_allowed):
		cost = simloopclosure(child,branch)
		odomUnc = branch[-1].odomUnc
	elif count == 1:	
		odomUnc = cov(map,edge,node.odomUnc)
	else: return -1,[]
	newcost = cost + odomUnc + distance
	branch.append(Node(child,odomUnc,distance,count + 1))

	return newcost,branch


def findVisitCount(name,branch):
	count = [vertex.visit_count for vertex in branch if (vertex.name == name)]
	if count == []: return 1
	return(count[-1])


class Branch():
	node_list = []
	visited = {}
	def __init__(self,map = None,init_node = None):
		if (map == None): return
		self.map = map
		self.node_list.append(init_node)
		for name in map:
			self.visited[name] = 0
		self.visited[init_node.name] += 1
		self.unexploredPenality = 1000	
		self.odomUncCost = 0
		self.distance = 0
		self.unexplored = self.unexploredPenality*len(map)
		self.total_visit_allowed = 2
		self.please_kill_me = False
	
	def copy_constructor(self,copy):
		self.map = copy.map
		self.node_list = deepcopy(copy.node_list)
		self.visited = deepcopy(copy.visited)
		self.unexploredPenality = copy.unexploredPenality
		self.odomUncCost = copy.odomUncCost
		self.distance = copy.distance
		self.unexplored = copy.unexplored
		self.total_visit_allowed = copy.total_visit_allowed
		self.please_kill_me = copy.please_kill_me

	def update(self,node_name):
		if(self.visited[node_name] == 0):
			self.unexplored -= self.unexploredPenality
		self.visited[node_name] += 1
		self.distance += self.updateDistance(node_name)
		odomUnc = self.updateOdom(node_name)
		self.odomUncCost += odomUnc
		node = Node(node_name,odomUnc,self.distance,self.visited[node_name])
		self.node_list.append(node)


	def updateDistance(self,node_name):
		return (self.map.get(self.node_list[-1].name).get(node_name))

	def updateOdom(self,node_name):
		count = self.visited[node_name]
		odomUnc = 0
		if (count > 1 and count <= self.total_visit_allowed):
			self.odomUncCost = simloopclosure(node_name,self.node_list)
			odomUnc = self.node_list[-1].odomUnc
		elif count == 1:
			edge = (self.node_list[-1].name,node_name)
			odomUnc = cov(self.map,edge,self.node_list[-1].odomUnc)
		else: self.please_kill_me = True	
		return odomUnc	
	
	def getTotalCost(self):
		return (self.distance + self.odomUncCost + self.unexplored)

	def getLatestNode(self):
		return self.node_list[-1]

	def printNodes(self):
		print [vertex.name for vertex in self.node_list]	


class Node():
	def __init__(self,name,odomUnc,distance,visit_count):
		self.name = name
		self.odomUnc = odomUnc
		self.visit_count = visit_count
		self.distance = distance


def simloopclosure(child,branch):
	cost = 0
	for vertex in reversed(branch):
		vertex.odomUnc = vertex.odomUnc*0.8
		cost += vertex.odomUnc
	#cost += branch[-1].distance
	return cost	

def BFS(map):
	bestScore = 1000000
	bestPath = []
	vertex = Node('A',0,0,1)
	branch = Branch(map,vertex)
	q = Q.PriorityQueue();
	q.put((branch.getTotalCost(),branch))

	
	while not q.empty():
		branch = None
		cost,branch = q.get()
		if bestScore > cost:
			bestScore = cost
			bestPath = branch
		children = map.get(branch.getLatestNode().name);
		for child in children:
		#	newcost,newBranch = processBranch(map,deepcopy(branch),cost,child)
			new_branch = None
			new_branch= Branch()
			new_branch.copy_constructor(branch)

			new_branch.update(child)

			if(new_branch.please_kill_me): continue
			###Kill on request
		#	if newcost == -1: continue
			q.put((new_branch.getTotalCost(),deepcopy(new_branch)))
		
	return bestScore,bestPath






def main():
	map = {'A': {'B':10.0},
           'B': {'A':10.0,'C':20.0, 'D':10.0},
           'C': {'B':20.0,'E':10.0},
           'D': {'B':10.0,'F':20.0,'E':10.0},
           'E': {'C':10.0,'D':10.0,'G':20.0},
           'F': {'D':20.0,'G':10.0},
           'G': {'E':20.0,'F':10.0}}
  	bestScore,bestPath = BFS(map)
  	print "bestScore: ", bestScore
  	bestPath.printNodes()
	return


if __name__ == "__main__":
    main()