from collections import deque
import heapq

items = [(4, "A"), (1, "B"), (2, "C")]
fifo = deque(items)
print("FIFO first:", fifo.popleft()[1])

waiting = []
for item in items:
    heapq.heappush(waiting, item)
while waiting:
    priority, node = heapq.heappop(waiting)
    print(priority, node)

