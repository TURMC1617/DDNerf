import itertools
from heapq import *

pq = []                         # list of entries arranged in a heap
counter = itertools.count()     # unique sequence count
#removes a duplicated task if there are any and adds + pushes a new task
def add_task(val):
    priority = 0
    count = next(counter)
    entry = [priority, count, val]
    heappush(pq, entry)

def priority_rank(pq):
    #cm = unit("cm")
    priority_0 = 5
    priority_1 = 10
    priority_2 = 15
    priority_3 = 20
    priority_4 = 25
    priority_5 = 30

    entry = pq[0]
    prio = entry[0]
    val = entry[2]
    dis = val[0]

    if dis <= priority_0:
        prio = 0
        entry[0] = prio
        

    if dis <= priority_1:
        prio = 1
        entry[0] = prio
        return 0


    if dis <= priority_2:
        prio = 2
        entry[0] = prio
        return 0


    if dis <= priority_3:
        prio = 3
        entry[0] = prio
        return 0

        
    if dis <= priority_4:
        prio = 4
        entry[0] = prio
        return 0

      
    if dis <= priority_5:
        prio = 5
        entry[0] = prio
        return 0
    
    if dis > 30:
        print("distance is greater than 30!")
        return 0


def d(coordinate):
    dis = ((coordinate[0]**2 + coordinate[1]**2)**(1/2.0))
    return dis

#tupple distance, angle
val = (11, 0)
val2 = (2, 0)
val3 = (3, 0)


#testing
add_task(val)
add_task(val2)
add_task(val3)

priority_rank(pq)
heappop(pq)             # pops the lowest priority val

print(pq)



                