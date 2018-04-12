import osmium
import csv
import svgwrite
import geopy.distance
import heapq
import time

SIZE = 5000


class OSM_handler(osmium.SimpleHandler):
    def __init__(self):
        osmium.SimpleHandler.__init__(self)
        self.nodes = {}  # id: [lat, lon]
        self.used_nodes = set()  # nodes in ways
        self.adjacency_list = {}  # id: set(id neighborhoods)
        self.amenities = {}
        self.max_latitude = -180.0
        self.max_longitude = -180.0
        self.min_latitude = 180.0
        self.min_longitude = 180.0

    def add_adjacency_node(self, key, value):
        if key in self.adjacency_list:
            self.adjacency_list[key].add(value)
        else:
            self.adjacency_list[key] = set()
            self.adjacency_list[key].add(value)

    def node(self, n):
        self.nodes[n.id] = [n.location.lat, n.location.lon]
        self.min_latitude = min(n.location.lat, self.min_latitude)
        self.max_latitude = max(n.location.lat, self.max_latitude)
        self.min_longitude = min(n.location.lon, self.min_longitude)
        self.max_longitude = max(n.location.lon, self.max_longitude)

        if n.tags.get("amenity") in ["restaurant"]:
            self.amenities[n.id] = [n.location.lat, n.location.lon]
            self.used_nodes.add(n.id)

    # проходим через все пути и строим список смежности
    def way(self, w):
        if w.tags.get("highway") in ["motorway", "trunk", "primary", "secondary", "tertiary", "unclassified",
                                     "residential", "living_street"]:
            count = len(w.nodes)
            if w.tags.get("oneway") == "yes":
                for i in range(0, count - 1):
                    self.add_adjacency_node(w.nodes[i].ref, w.nodes[i + 1].ref)
                    self.used_nodes.add(w.nodes[i + 1].ref)
                self.used_nodes.add(w.nodes[0].ref)

            else:
                for i in range(1, count - 1):
                    self.add_adjacency_node(w.nodes[i].ref, w.nodes[i + 1].ref)
                    self.add_adjacency_node(w.nodes[i].ref, w.nodes[i - 1].ref)
                    self.used_nodes.add(w.nodes[i].ref)
                self.add_adjacency_node(w.nodes[0].ref, w.nodes[1].ref)
                self.add_adjacency_node(w.nodes[count - 1].ref, w.nodes[count - 2].ref)
                self.used_nodes.add(w.nodes[0].ref)

    def get_bounds(self):
        return self.max_latitude, self.min_latitude, self.max_longitude, self.min_longitude

    def remove_unused_nodes(self):
        nodes_temp = self.nodes.copy()
        for node in nodes_temp:
            if node not in self.used_nodes:
                self.nodes.pop(node, None)
        return self.nodes


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def flip_and_center_point(point):
    return SIZE - (max_lon - float(point[1])) / scale_lon, \
           (max_lat - float(point[0])) / scale_lat


def add_way_to_amenities(handler, amenities):
    min_dist = {}
    for am in amenities:
        min_dist[am] = 400000
    near_node = {}

    for node in handler.nodes:
        if node not in handler.adjacency_list:
            continue
        distance = {}
        for am in amenities:
            # distance[am] = geopy.distance.vincenty((handler.nodes[node][0], handler.nodes[node][1]),
            #                                        (handler.nodes[am][0], handler.nodes[am][1])).km
            distance[am] = (handler.nodes[node][0] - handler.nodes[am][0]) * (
                handler.nodes[node][0] - handler.nodes[am][0]) + (
                                                                     handler.nodes[node][1] - handler.nodes[am][1]) * (
                                                                     handler.nodes[node][1] - handler.nodes[am][1])
        for d in distance:
            if min_dist[d] > distance[d]:
                min_dist[d] = distance[d]
                near_node[d] = node
    for node in near_node:
        x1, y1 = flip_and_center_point(h.nodes[node])
        x2, y2 = flip_and_center_point(h.nodes[near_node[node]])
        draw.add(draw.line((x1, y1), (x2, y2), stroke=svgwrite.rgb(10, 10, 10, '%')))
        handler.add_adjacency_node(near_node[node], node)
    draw.save()


def save_adjacency_list(adjacency_list):
    # сохраняем список смежности в .csv
    print("Создание списка смежности...")
    with open('mapAL.csv', 'w') as printer_map_AL:
        out_map_AL = csv.writer(printer_map_AL)
        out_map_AL.writerows(adjacency_list.items())


def save_adjacency_matrix(used_nodes, adjacency_list):
    # сохраняем матрицу смежности в .csv
    print("Создание матрицы смежности...")
    with open('mapAM.csv', 'w') as printer_map_AM:
        out_map_AM = csv.writer(printer_map_AM)
        list_nodes = list(used_nodes)
        count_nodes = len(list_nodes)
        print("Используемых узлов: %d" % count_nodes)
        out_map_AM.writerow([''] + list_nodes)
        for node in list_nodes:
            adjacent = [0 for i in range(count_nodes)]
            if node in adjacency_list:
                for value in adjacency_list[node]:
                    adjacent[list_nodes.index(value)] = 1
            out_map_AM.writerow([node] + list(adjacent))


def create_map_csv(handler, drawing, size):
    # отрисовка графа
    print("Draw the graph...")

    adj_list = handler.adjacency_list
    for node in handler.adjacency_list:
        if node not in handler.nodes:
            continue
        x = size - (max_lon - float(handler.nodes[node][1])) / scale_lon
        y = (max_lat - float(handler.nodes[node][0])) / scale_lat
        drawing.add(drawing.circle((x, y), 1.5))

        for n in adj_list[node]:
            if n not in handler.nodes:
                continue
            x1, y1 = flip_and_center_point(handler.nodes[node])
            x2, y2 = flip_and_center_point(handler.nodes[n])
            drawing.add(drawing.line((x1, y1), (x2, y2), stroke=svgwrite.rgb(10, 10, 10, '%')))

    for amenity in handler.amenities:
        x, y = flip_and_center_point(handler.amenities[amenity])
        drawing.add(drawing.circle((x, y), r=2, fill="yellow"))
    print("Amenities: {0}".format(len(handler.amenities)))

    drawing.save()


def enter_coordinates():
    true_coordinates = False
    lat, lon = 0.0, 0.0
    while not true_coordinates:
        print('Введите координаты в диапазоне:\nLatitude ({0:.6}, {1:.6}); \nLongitude ({2:.6}, {3:.6})'
              .format(min_lat + 0.1, max_lat - 0.1, min_lon + 0.1, max_lon - 0.1))
        lat, lon = map(float, input().split())
        if min_lat + 0.1 < lat < max_lat - 0.1 and min_lon + 0.1 < lon < max_lon - 0.1:
            true_coordinates = True

    return lat, lon


def add_start_node(handler, lat, lon, draw_node=False, id_node=0):
    id_near_node = -1
    min_distance = 100000

    for node in handler.adjacency_list:
        if node not in handler.nodes:
            continue
        # distance = geopy.distance.vincenty((lat, lon), (handler.nodes[node][0], handler.nodes[node][1])).km
        distance = (lat - handler.nodes[node][0]) ** 2 + (lon - handler.nodes[node][1]) ** 2
        if min_distance > distance:
            min_distance = distance
            id_near_node = node

    handler.nodes[id_node] = [lat, lon]
    handler.add_adjacency_node(id_node, id_near_node)
    if draw_node:
        draw.add(draw.circle((flip_and_center_point([lat, lon])), r=15, color="crimson"))
        draw.save()


def del_start_node(handler, id_node=0):
    if id_node in handler.nodes:
        handler.nodes.pop(id_node)
    if id_node in handler.adjacency_list:
        handler.adjacency_list.pop(id_node)


def sqr_distance(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def chebishev(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def dijkstra(handler, start, goals: []):
    goals = goals.copy()
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {start: None}
    distance = {start: 0}
    came_goals = []

    while not frontier.empty() and len(goals) > 0:
        current = frontier.get()
        if current in goals:
            came_goals.append(current)
            goals.remove(current)
            continue
        if current not in handler.adjacency_list:
            continue

        for node in handler.adjacency_list[current]:
            if node not in handler.nodes:
                continue
            new_cost = distance[current] + (handler.nodes[current][0] - handler.nodes[node][0]) ** 2 + \
                       (handler.nodes[current][1] - handler.nodes[node][1]) ** 2
            if node not in distance or new_cost < distance[node]:
                distance[node] = new_cost
                frontier.put(node, new_cost)
                came_from[node] = current
    # print("Goals: {0}".format(len(came_goals)))
    # print(came_from)
    # c = 1
    # for node in came_from:
    #     if node == start:
    #         continue
    #     c += 0.0005
    #     x1, y1 = flip_and_center_point(h.nodes[node])
    #     x2, y2 = flip_and_center_point(h.nodes[came_from[node]])
    #     draw.add(draw.line((x1, y1), (x2, y2), stroke="green", stroke_width=1.5 + c, fill="none"))
    # draw.save()
    return came_from, distance, came_goals


def levit(handler, start, goals: []):
    goals = goals.copy()
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {start: None}
    distances = {start: 0.0}
    came_goals = []

    for n in handler.nodes.keys():
        if n != start:
            distances[n] = 100.0  # inf

    while not frontier.empty():
        current = frontier.get()
        if current in goals:
            came_goals.append(current)
            continue
        if current not in handler.adjacency_list:
            continue

        for node in handler.adjacency_list[current]:
            if node not in handler.nodes:
                continue
            distance = distances[current] + (handler.nodes[current][0] - handler.nodes[node][0]) ** 2 + \
                       (handler.nodes[current][1] - handler.nodes[node][1]) ** 2
            if distance < distances[node]:
                distances[node] = distance
                came_from[node] = current
                frontier.put(node, distance)
    # print("Goals: {0}".format(len(came_goals)))
    return came_from, distances, came_goals


def find_distance(a, b, m='euclid'):
    if m == 'manhattan':
        return manhattan(a, b)
    elif m == 'chebishev':
        return chebishev(a, b)
    else:
        return sqr_distance(a, b) ** 0.5


def a_star(handler, start, goal, heuristic="euclid"):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {start: None}
    distances = {start: 0}
    came_to_goal = False

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            came_to_goal = True
            break
        if current not in handler.adjacency_list:
            continue

        for node in handler.adjacency_list[current]:
            if node not in handler.nodes:
                continue
            new_cost = distances[current] + (handler.nodes[current][0] - handler.nodes[node][0]) ** 2 + \
                       (handler.nodes[current][1] - handler.nodes[node][1]) ** 2
            if node not in distances or new_cost < distances[node]:
                distances[node] = new_cost
                priority = new_cost + find_distance(handler.nodes[goal], handler.nodes[node], heuristic)
                frontier.put(node, priority)
                came_from[node] = current

    return came_from, distances, came_to_goal


def reconstruct_path(came_from, start, goal, nodes):
    distance = 0
    current = goal
    path = []
    while current != start:
        path.append(current)
        distance += geopy.distance.vincenty((nodes[current][0], nodes[current][1]),
                                            (nodes[came_from[current]][0], nodes[came_from[current]][1])).km
        current = came_from[current]

    path.append(start)
    path.reverse()
    return path, distance


def draw_path(path, drawing, color, width, nodes):
    for i in range(0, len(path) - 1):
        x1, y1 = flip_and_center_point(nodes[path[i]])
        x2, y2 = flip_and_center_point(nodes[path[i + 1]])
        drawing.add(drawing.line((x1, y1), (x2, y2), stroke=color, stroke_width=width, fill="none"))
    x, y = flip_and_center_point(nodes[path.pop()])
    drawing.add(drawing.circle((x, y), r=6, fill="gold"))


def find_path(handler, goals, algorithm, start=0, draw_paths=False, heuristic='euclid'):
    # print("Ищем дорогу...")

    distances = {}
    if algorithm == "dijkstra":
        came, _distance, _goals = dijkstra(handler, start, goals)
        for g in _goals:
            way, d = reconstruct_path(came, start, g, handler.nodes)
            distances[g] = d
            if draw_paths:
                draw_path(way, draw, "green", 3, handler.nodes)
        if len(_goals) > 0 and draw_paths:
            way, d = reconstruct_path(came, start, _goals[0], handler.nodes)
            draw_path(way, draw, "red", 4, handler.nodes)
    elif algorithm == "levit":
        came, _distance, _goals = levit(handler, start, goals)
        for g in _goals:
            way, d = reconstruct_path(came, start, g, handler.nodes)
            distances[g] = d
            if draw_paths:
                draw_path(way, draw, "aqua", 3, handler.nodes)
        if len(_goals) > 0 and draw_paths:
            way, d = reconstruct_path(came, start, _goals[0], handler.nodes)
            draw_path(way, draw, "red", 4, handler.nodes)
    elif algorithm == "a_star":
        temp_came = {}
        dist = 400000
        near_came = None
        count = len(goals)
        for g in goals:
            came, _distance, came_to_goal = a_star(handler, start, g, heuristic)
            if not came_to_goal:
                count -= 1
                continue
            if dist > _distance[g]:
                temp_came = came.copy()
                dist = _distance[g]
                near_came = g
            way, d = reconstruct_path(came, start, g, handler.nodes)
            distances[g] = d
            if draw_paths:
                draw_path(way, draw, "navy", 3, handler.nodes)
        if len(temp_came) > 0 and draw_paths:
            way, d = reconstruct_path(temp_came, start, near_came, handler.nodes)
            draw_path(way, draw, "red", 4, handler.nodes)
            # print("Goals: {0}".format(count))
    if draw_paths:
        draw.save()
    return distances


def test(handler, count_coord=3):
    ams = list(h.amenities)[20:30]
    print("Way to: {0} restaurant".format(len(ams)))
    lats = []
    for i in range(count_coord):
        lats.append(min_lat + 0.1 + (max_lat - min_lat - 0.1) / count_coord * i)
    lons = []
    for i in range(count_coord):
        lons.append(min_lon + 0.1 + (max_lon - min_lon - 0.1) / count_coord * i)

    time_dijkstra = 0
    time_levit = 0
    time_a_star = 0
    a_star_minus_dijkstra = {a: 0 for a in ams}
    avg_dist = 0

    for lt in lats:
        for ln in lons:
            add_start_node(handler, lt, ln, True)

            t0 = time.time()
            distances_dijkstra = find_path(handler, ams.copy(), "dijkstra", 0, False)
            t1 = time.time()
            time_dijkstra += t1 - t0

            t0 = time.time()
            distances_levit = find_path(handler, ams.copy(), "levit", 0, False)
            t1 = time.time()
            time_levit += t1 - t0

            t0 = time.time()
            distances_a_star = find_path(handler, ams.copy(), "a_star", 0, False, 'euclid')
            t1 = time.time()
            time_a_star += t1 - t0
            # находим среднее расстояние до цели
            number_goals = 0
            distance_goals = 0
            for a in ams:
                if a in distances_dijkstra and a in distances_a_star:
                    a_star_minus_dijkstra[a] += distances_a_star[a] - distances_dijkstra[a]
                    number_goals += 1
                    distance_goals += distances_a_star[a]
            if number_goals > 0:
                avg_dist += distance_goals / number_goals

            del_start_node(handler)
    avg_dist /= count_coord ** 2
    print("Time Dijkstra: {0}".format(time_dijkstra))
    print("Time Levit: {0}".format(time_levit))
    print("Time A_star: {0}".format(time_a_star))
    print("Average distance: {0:.10}km".format(avg_dist))
    print("Average time: {0:.6} minutes".format(avg_dist * 60 / 40))
    print("\nDifference of distances: ")
    print(a_star_minus_dijkstra)
    print("Diff = {0:.6}".format(sum(a_star_minus_dijkstra.values(), 0)))


def test_heuristic(handler, count_coord=3):
    ams = list(h.amenities)[20:30]
    print("Way to: {0} restaurant".format(len(ams)))
    lats = []
    for i in range(count_coord):
        lats.append(min_lat + 0.1 + (max_lat - min_lat - 0.1) / count_coord * i)
    lons = []
    for i in range(count_coord):
        lons.append(min_lon + 0.1 + (max_lon - min_lon - 0.1) / count_coord * i)

    time_a_star_eu = 0
    time_a_star_ma = 0
    time_a_star_ch = 0
    avg_dist_eu = 0
    avg_dist_ma = 0
    avg_dist_ch = 0

    for lt in lats:
        for ln in lons:
            add_start_node(handler, lt, ln, True)

            t0 = time.time()
            distances_a_star = find_path(handler, ams.copy(), "a_star", 0, False, 'euclid')
            t1 = time.time()
            time_a_star_eu += t1 - t0
            number_goals = 0
            distance_goals = 0
            for a in ams:
                if a in distances_a_star:
                    number_goals += 1
                    distance_goals += distances_a_star[a]
            if number_goals > 0:
                avg_dist_eu += distance_goals / number_goals

            t0 = time.time()
            distances_a_star = find_path(handler, ams.copy(), "a_star", 0, False, 'manhattan')
            t1 = time.time()
            time_a_star_ma += t1 - t0
            number_goals = 0
            distance_goals = 0
            for a in ams:
                if a in distances_a_star:
                    number_goals += 1
                    distance_goals += distances_a_star[a]
            if number_goals > 0:
                avg_dist_ma += distance_goals / number_goals

            t0 = time.time()
            distances_a_star = find_path(handler, ams.copy(), "a_star", 0, False, 'chebishev')
            t1 = time.time()
            time_a_star_ch += t1 - t0
            number_goals = 0
            distance_goals = 0
            for a in ams:
                if a in distances_a_star:
                    number_goals += 1
                    distance_goals += distances_a_star[a]
            if number_goals > 0:
                avg_dist_ch += distance_goals / number_goals

            del_start_node(handler)
    avg_dist_eu /= count_coord ** 2
    avg_dist_ma /= count_coord ** 2
    avg_dist_ch /= count_coord ** 2

    print("Euclidean distance: {0:.6}".format(avg_dist_eu))
    print("Manhattan distance: {0:.6}".format(avg_dist_ma))
    print("Chebyshev distance: {0:.6}".format(avg_dist_ch))
    print("Euclidean time: {0:.6}".format(time_a_star_eu))
    print("Manhattan time: {0:.6}".format(time_a_star_ma))
    print("Chebyshev time: {0:.6}".format(time_a_star_ch))


def make(handler):
    lt, ln = enter_coordinates()
    add_start_node(handler, lt, ln, True)
    amen = list(handler.amenities)[20:30]
    ways = {}
    for g in amen:
        came, _distance, came_to_goal = a_star(handler, 0, g)
        if not came_to_goal:
            continue
        way, d = reconstruct_path(came, 0, g, handler.nodes)
        draw_path(way, draw, "navy", 3, handler.nodes)
        draw.save()
        ways[int(way[-1])] = way
    with open('map_with_way.csv', 'w') as printer_map_ww:
        out_map_ww = csv.writer(printer_map_ww)
        out_map_ww.writerows(ways.items())
    del_start_node(h)


if __name__ == '__main__':
    draw = svgwrite.Drawing('mapGraph.svg', size=('%dpx' % SIZE, '%dpx' % SIZE))
    h = OSM_handler()
    h.apply_file("map.osm")
    h.nodes = h.remove_unused_nodes()

    max_lat, min_lat, max_lon, min_lon = h.get_bounds()
    scale_lat = (max_lat - min_lat) / SIZE
    scale_lon = (max_lon - min_lon) / SIZE

    # save_adjacency_list(h.adjacency_list)
    # save_adjacency_matrix(h.used_nodes, h.adjacency_list)
    create_map_csv(h, draw, SIZE)
    add_way_to_amenities(h, list(h.amenities))
    # test(h, 10)
    # test_heuristic(h, 10)
    make(h)
