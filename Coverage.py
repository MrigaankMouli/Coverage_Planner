import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
import pyproj
import json

def utm_to_gps(utm_proj, utm_x, utm_y):
    gps_proj = pyproj.Proj(proj='latlong', datum='WGS84')
    lon, lat = pyproj.transform(utm_proj, gps_proj, utm_x, utm_y)
    return lat, lon

def generate_coverage_waypoints(json_file_path, cell_width, cell_height):
    # Read boundary coordinates from JSON file
    with open(json_file_path, 'r') as json_file:
        data = json.load(json_file)
    latitudes = [point['latitude'] for point in data['waypoints']]
    longitudes = [point['longitude'] for point in data['waypoints']]
    points = list(zip(longitudes, latitudes))

    utm_proj = pyproj.Proj(proj='utm', zone=18, ellps='WGS84')
    utm_points = [utm_proj(lon, lat) for lon, lat in points]
    polygon = Polygon(utm_points)

    # Calculate the minimum and maximum coordinates of the polygon
    min_x, min_y, max_x, max_y = polygon.bounds

    # Calculate the number of cells in the x and y directions
    num_cells_x = int(np.ceil((max_x - min_x) / cell_width))
    num_cells_y = int(np.ceil((max_y - min_y) / cell_height))

    centroids = []
    for i in range(num_cells_x):
        for j in range(num_cells_y):
            cell_x = min_x + i * cell_width
            cell_y = min_y + j * cell_height
            cell = Polygon([(cell_x, cell_y), (cell_x + cell_width, cell_y), (cell_x + cell_width, cell_y + cell_height), (cell_x, cell_y + cell_height)])
            if polygon.intersects(cell):
                # Check if the cell is fully within the polygon
                if polygon.contains(cell):
                    centroids.append(cell.centroid)
                else:
                    # Clip the cell to the polygon boundaries
                    clipped_cell = cell.intersection(polygon)
                    centroids.append(clipped_cell.centroid)

    fig, ax = plt.subplots()
    for point in centroids:
        ax.plot(point.x, point.y, 'ro')
    x, y = polygon.exterior.xy
    ax.plot(x, y, color='red')
    ax.set_aspect('equal')
    plt.show()

    gps_centroids = [utm_to_gps(utm_proj, point.x, point.y) for point in centroids]
    gps_centroids_dict = {"waypoints": [{"latitude": lat, "longitude": lon, "altitude": 45} for lat, lon in gps_centroids]}

    return gps_centroids_dict

# Example usage
json_file_path = '../missions/8april/msap/coverage_boundary.json'
gps_centroids_dict = generate_coverage_waypoints(
    json_file_path,
    cell_width=15,
    cell_height=10
)

# Write the GPS centroids to a JSON file
output_json_file_path = '../missions/8april/msap/coverage_waypoints.json'
with open(output_json_file_path, 'w') as output_json_file:
    json.dump(gps_centroids_dict, output_json_file, indent=4)