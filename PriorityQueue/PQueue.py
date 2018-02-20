import Queue
from fullfrontal3
# unique sequence count
#removes a duplicated task if there are any and adds + pushes a new task
pq = Queue.PriorityQueue()

global tupls = ()

def populate_queue(depth, angle):
    tupls = (depth, angle)
    pq.put(tupls)

def popping():
    return tupls.get()


        

def d(coordinate):
    dis = ((coordinate[0]**2 + coordinate[1]**2)**(1/2.0))
    return dis

#tupple distance, angle


priority_rank(pq)
heappop(pq)             # pops the lowest priority val

print(pq)



                
