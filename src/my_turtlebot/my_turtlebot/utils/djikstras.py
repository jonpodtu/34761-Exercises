import heapq

def dijkstra(edges, start_idx, goal_idx):
    queue = [(0, start_idx)]
    distances = {node: float("infinity") for node in edges}
    distances[start_idx] = 0
    previous_nodes = {node: None for node in edges}

    while queue:
        # Pop the node with the smallest distance from the queue
        current_distance, current_node = heapq.heappop(queue)

        # If this node has already been visited, skip it
        if current_distance > distances[current_node]:
            continue

        # For each neighbor of the current node
        for neighbor, weight in edges[current_node]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue,(distance, neighbor))

        # Reconstruct the shortest path
        path = []
        current_node = goal_idx
        while current_node is not None:
            path.append(current_node)
            current_node = previous_nodes[current_node]
        path.reverse()
    
    return path, distances
