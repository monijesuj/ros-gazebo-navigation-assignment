"""ROS-independent occupancy planning and local WGS84 conversion."""
import heapq
import math
import numpy as np


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def geodetic_to_xy(latitude, longitude, datum):
    """Local ENU tangent plane; intended for this metre-scale world only."""
    lat0, lon0 = map(math.radians, datum[:2])
    a, e2 = 6378137.0, 6.69437999014e-3
    d = 1 - e2 * math.sin(lat0) ** 2
    n, m = a / math.sqrt(d), a * (1 - e2) / d ** 1.5
    return (math.radians(longitude) - lon0) * n * math.cos(lat0), (math.radians(latitude) - lat0) * m


class Grid:
    def __init__(self, config):
        self.xmin, self.xmax, self.ymin, self.ymax = config['bounds']
        self.resolution = config['resolution']
        self.width = round((self.xmax - self.xmin) / self.resolution)
        self.height = round((self.ymax - self.ymin) / self.resolution)
        self.radius = config['robot_radius'] + config['safety_margin']
        self.static = np.zeros((self.height, self.width), dtype=bool)
        self.dynamic_until = np.zeros_like(self.static, dtype=float)
        for o in config['obstacles']:
            for y in range(self.height):
                for x in range(self.width):
                    wx, wy = self.world((x, y))
                    if abs(wx - o['x']) <= o['sx'] / 2 + self.resolution/2 and abs(wy - o['y']) <= o['sy']/2 + self.resolution/2:
                        self.static[y, x] = True

    def cell(self, point):
        return math.floor((point[0] - self.xmin) / self.resolution), math.floor((point[1] - self.ymin) / self.resolution)

    def world(self, cell):
        return self.xmin + (cell[0] + .5) * self.resolution, self.ymin + (cell[1] + .5) * self.resolution

    def inside(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def observe(self, point, now):
        cell = self.cell(point)
        if self.inside(cell):
            x, y = cell
            self.dynamic_until[y, x] = now + 3.0

    def occupied(self, now):
        return self.static | (self.dynamic_until > now)

    def inflated(self, now):
        occupied = self.occupied(now)
        blocked = occupied.copy()
        count = math.ceil(self.radius / self.resolution)
        for dy in range(-count, count + 1):
            for dx in range(-count, count + 1):
                if math.hypot(dx, dy) * self.resolution > self.radius + self.resolution * .71:
                    continue
                ys = slice(max(0, dy), min(self.height, self.height + dy))
                xs = slice(max(0, dx), min(self.width, self.width + dx))
                blocked[ys, xs] |= occupied[slice(max(0, -dy), min(self.height, self.height - dy)), slice(max(0, -dx), min(self.width, self.width - dx))]
        # Keep the entire robot within the map bounds.
        blocked[:count, :] = blocked[-count:, :] = True
        blocked[:, :count] = blocked[:, -count:] = True
        return blocked

    def line_free(self, a, b, blocked):
        steps = max(1, math.ceil(math.dist(a, b) / (self.resolution / 3)))
        for i in range(steps + 1):
            t = i / steps
            cell = self.cell((a[0] + t*(b[0]-a[0]), a[1] + t*(b[1]-a[1])))
            if not self.inside(cell) or blocked[cell[1], cell[0]]:
                return False
        return True

    def plan(self, start, goal, now):
        blocked = self.inflated(now)
        source, target = self.cell(start), self.cell(goal)
        if not self.inside(source) or not self.inside(target):
            return []
        if blocked[source[1], source[0]] or blocked[target[1], target[0]]:
            return []
        frontier = [(math.dist(source, target), 0., source)]
        costs, parent = {source: 0.}, {}
        while frontier:
            _, cost, cur = heapq.heappop(frontier)
            if cost > costs[cur]:
                continue
            if cur == target:
                cells = [cur]
                while cur in parent:
                    cur = parent[cur]
                    cells.append(cur)
                raw = [start] + [self.world(c) for c in reversed(cells)][1:-1] + [goal]
                # Greedy visibility pruning, checked against the inflated grid.
                path, i = [raw[0]], 0
                while i < len(raw)-1:
                    j = len(raw)-1
                    while j > i+1 and not self.line_free(raw[i], raw[j], blocked):
                        j -= 1
                    path.append(raw[j]); i = j
                return path
            x, y = cur
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = x+dx, y+dy
                if not self.inside(nxt) or blocked[nxt[1], nxt[0]]:
                    continue
                if dx and dy and (blocked[y, x+dx] or blocked[y+dy, x]):
                    continue
                new_cost = cost + math.hypot(dx,dy)
                if new_cost < costs.get(nxt, float('inf')):
                    costs[nxt] = new_cost; parent[nxt] = cur
                    heapq.heappush(frontier, (new_cost + math.dist(nxt,target), new_cost, nxt))
        return []


def clearance(point, obstacles):
    """Distance from robot centre to closest axis-aligned obstacle rectangle."""
    return min(math.hypot(max(abs(point[0]-o['x'])-o['sx']/2,0), max(abs(point[1]-o['y'])-o['sy']/2,0)) for o in obstacles)
