import numpy
import networkx as nx
import heapq, operator
from scipy.spatial.distance import cdist

herb_lower_limit = numpy.array([0.54159265359, -2.00, -2.8, -0.9, -4.76, -1.6, -3.0])
herb_upper_limit = numpy.array([5.74159265359,  2.00,  2.8,  3.1,  1.24,  1.6,  3.0])
herb_limit_difference = herb_upper_limit - herb_lower_limit

startState = [5.65, -1.50, -0.26, 1.96, -1.15, 0.87, -1.43]
goalState = [5.02, -0.24, -0.12, 0.87, 1.21, 0.34, 3.47]

def halton_sequence_value(index, base):

    result = 0
    f = 1

    while index > 0:
        f = f*1.0/base
        result = result + f*(index % base)
        index = index/base

    return result

def wrap_around(coordinate):

    for i in range(numpy.size(coordinate)):
        if coordinate[i] > 1.0:
            coordinate[i] = coordinate[i] - 1.0
        if coordinate[i] < 0:
            coordinate[i] = 1.0 + coordinate[i]

    return coordinate

def make_herb_halton_graph(n, r):

    G = nx.Graph()

    bases = [2,3,5,7,11,13,17]
    offset = numpy.random.random_sample(7)*0
    position = {i-1:wrap_around(numpy.array([halton_sequence_value(i,base) for base in bases])) for i in range(1,n+1)}
    scaled_pos = {i: herb_lower_limit + numpy.multiply(position[i], herb_limit_difference) for i in position.keys()}

    halton = []
    for i in range(n):
        halton.append(scaled_pos[i])
        state = str(scaled_pos[i][0]) + ' ' + str(scaled_pos[i][1])+' '+str(scaled_pos[i][2])+' '+str(scaled_pos[i][3]) + ' ' + str(scaled_pos[i][4])+' '+str(scaled_pos[i][5])+' '+str(scaled_pos[i][6])
        G.add_node(i, state = state)

    for i in range(n-1):
        if i%100 == 0:
            print i
        for j in range(i+1,n):
            dist = numpy.linalg.norm(scaled_pos[i] - scaled_pos[j])
            if dist < r:
                G.add_edge(i, j)

    G.add_node(n, state = str(startState[0]) + ' ' + str(startState[1])+' '+str(startState[2])+' '+str(startState[3]) + ' ' + str(startState[4])+' '+str(startState[5])+' '+str(startState[6]))
    G.add_node(n+1, state = str(goalState[0]) + ' ' + str(goalState[1])+' '+str(goalState[2])+' '+str(goalState[3]) + ' ' + str(goalState[4])+' '+str(goalState[5])+' '+str(goalState[6]))

    pair_distances = cdist([startState,goalState],halton)

    knn = 15
    for i in range(2):
         indices = zip(*heapq.nsmallest(knn, enumerate(pair_distances[i]), key=operator.itemgetter(1)))[0]
         for j in indices:
             if i == j:
                 continue
             G.add_edge(i+n, j)

    print('Average degree:', G.number_of_edges() * 2.0 / n)
    return G

def main(n=500, r=0.35):
    G = make_herb_halton_graph(n, r)
    fileName = '../graphs/herb_halton_{}_r{}.graphml'.format(n, int(r*100))
    nx.write_graphml(G, fileName)
    print(fileName)

if __name__ == '__main__':
    main()
