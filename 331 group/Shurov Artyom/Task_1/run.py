import osmium
import csv
import svgwrite


class CounterHandler(osmium.SimpleHandler):
    def __init__(self):
        osmium.SimpleHandler.__init__(self)
        self.nodes = {}  # id: [lat, lon]
        self.usedNodes = set()  # nodes in ways
        self.adjacencyList = {}

    def addAdjacencyNode(self, key, value):
        if key in self.adjacencyList:
            self.adjacencyList[key].add(value)
        else:
            self.adjacencyList[key] = set()
            self.adjacencyList[key].add(value)

    def node(self, n):
        self.nodes[n.id] = [n.location.lat, n.location.lon]

    # проходим через все пути и строим список смежности
    def way(self, w):
        if w.tags.get("highway") in ["motorway", "trunk", "primary", "secondary", "tertiary", "unclassified",
                                     "residential", "living_street"]:
            count = len(w.nodes)
            if w.tags.get("oneway") == "yes":
                for i in range(0, count - 1):
                    self.addAdjacencyNode(w.nodes[i].ref, w.nodes[i + 1].ref)
                    self.usedNodes.add(w.nodes[i + 1].ref)
                self.usedNodes.add(w.nodes[0].ref)

            else:
                for i in range(1, count - 1):
                    self.addAdjacencyNode(w.nodes[i].ref, w.nodes[i + 1].ref)
                    self.addAdjacencyNode(w.nodes[i].ref, w.nodes[i - 1].ref)
                    self.usedNodes.add(w.nodes[i].ref)
                self.addAdjacencyNode(w.nodes[0].ref, w.nodes[1].ref)
                self.addAdjacencyNode(w.nodes[count - 1].ref, w.nodes[count - 2].ref)
                self.usedNodes.add(w.nodes[0].ref)
                self.usedNodes.add(w.nodes[count - 1].ref)


if __name__ == '__main__':

    h = CounterHandler()
    h.apply_file("map.osm")

    listNodes = list(h.usedNodes)
    listNodes.sort()
    countNodes = len(listNodes)

    # сохраняем список смежности в .csv
    print("Создание списка смежности...")
    with open('mapAL.csv', 'w') as printer:
        out = csv.writer(printer)
        out.writerows(h.adjacencyList.items())

    # сохраняем матрицу смежности в .csv
    print("Создание матрицы смежности...")
    with open('mapAM.csv', 'w') as printer:
        out = csv.writer(printer)
        print("Используемых узлов: %d" % countNodes)
        out.writerow([''] + listNodes)
        for node in listNodes:
            adjacent = [0 for i in range(countNodes)]
            if node in h.adjacencyList:
                for value in h.adjacencyList[node]:
                    adjacent[listNodes.index(value)] = 1
            out.writerow([node] + list(adjacent))

    # отрисовка графа
    print("Отрисовка графа...")
    # нахождение границ
    maxLat = 0.0
    maxLon = 0.0
    minLat = 180.0
    minLon = 180.0

    for node in listNodes:
        if node not in h.nodes:
            continue
        lat, lon = h.nodes[node][0], h.nodes[node][1]

        if minLat > lat:
            minLat = lat
        elif maxLat < lat:
            maxLat = lat

        if minLon > lon:
            minLon = lon
        elif maxLon < lon:
            maxLon = lon

    size = 5000
    scaleLat = (maxLat - minLat) / size
    scaleLon = (maxLon - minLon) / size
    graph = svgwrite.Drawing('mapGraph.svg', size=('%dpx' % size, '%dpx' % size))
    for node in h.adjacencyList:
        if node not in h.nodes:
            continue
        graph.add(graph.circle(
            (size - (maxLon - float(h.nodes[node][1])) / scaleLon, (maxLat - float(h.nodes[node][0])) / scaleLat), 1.5))
        for n in h.adjacencyList[node]:
            if n not in h.nodes:
                continue
            graph.add(graph.circle((size - (maxLon - float(h.nodes[n][1])) / scaleLon,
                                    (maxLat - float(h.nodes[n][0])) / scaleLat), 1.5))
            graph.add(graph.line((size - (maxLon - float(h.nodes[node][1])) / scaleLon,
                                  (maxLat - float(h.nodes[node][0])) / scaleLat),
                                 (size - (maxLon - float(h.nodes[n][1])) / scaleLon,
                                  (maxLat - float(h.nodes[n][0])) / scaleLat),
                                 stroke=svgwrite.rgb(10, 10, 10, '%')))
    graph.save()
