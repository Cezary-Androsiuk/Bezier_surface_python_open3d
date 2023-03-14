import numpy as np
import open3d as o3d
import copy

from scipy.spatial import KDTree


# NOT EFFICIENT AND OLD CODE
# def nck(n, k):
#     return np.math.factorial(n) / (np.math.factorial(k) * np.math.factorial(n-k))
#
# def berstein(n, i, step):
#     # 0 ≤ step ≤ 1
#     return nck(n, i) * ((1-step) ** (n-i)) * step ** i
#
# def computePointFunctions(control_points_array, step_x, step_y):
#     x = 0
#     y = 0
#     z = 0
#
#     i = 0
#     n = len(control_points_array)-1
#     for control_points_vector in control_points_array:
#         j = 0
#         m = len(control_points_vector)-1
#         for P in control_points_vector:
#             x += berstein(n, i, step_x) * berstein(m, j, step_y) * P[0]
#             y += berstein(n, i, step_x) * berstein(m, j, step_y) * P[1]
#             z += berstein(n, i, step_x) * berstein(m, j, step_y) * P[2]
#             j += 1
#         i += 1
#
#     return x, y, z

def computePointFunction(control_points_array, step_x, step_y):
    # P[0][0][] * (1 * ((1 - step_x) ** 3) * step_x ** 0) * (1 * ((1 - step_y) ** 3) * step_y ** 0) + \
    # P[0][1][] * (3 * ((1 - step_x) ** 2) * step_x ** 1) * (1 * ((1 - step_y) ** 3) * step_y ** 0) + \
    # P[0][2][] * (3 * ((1 - step_x) ** 1) * step_x ** 2) * (1 * ((1 - step_y) ** 3) * step_y ** 0) + \
    # P[0][3][] * (1 * ((1 - step_x) ** 0) * step_x ** 3) * (1 * ((1 - step_y) ** 3) * step_y ** 0)\
    # + \
    # P[1][0][] * (1 * ((1 - step_x) ** 3) * step_x ** 0) * (3 * ((1 - step_y) ** 2) * step_y ** 1) + \
    # P[1][1][] * (3 * ((1 - step_x) ** 2) * step_x ** 1) * (3 * ((1 - step_y) ** 2) * step_y ** 1) + \
    # P[1][2][] * (3 * ((1 - step_x) ** 1) * step_x ** 2) * (3 * ((1 - step_y) ** 2) * step_y ** 1) + \
    # P[1][3][] * (1 * ((1 - step_x) ** 0) * step_x ** 3) * (3 * ((1 - step_y) ** 2) * step_y ** 1)\
    # + \
    # P[2][0][] * (1 * ((1 - step_x) ** 3) * step_x ** 0) * (3 * ((1 - step_y) ** 1) * step_y ** 2) + \
    # P[2][1][] * (3 * ((1 - step_x) ** 2) * step_x ** 1) * (3 * ((1 - step_y) ** 1) * step_y ** 2) + \
    # P[2][2][] * (3 * ((1 - step_x) ** 1) * step_x ** 2) * (3 * ((1 - step_y) ** 1) * step_y ** 2) + \
    # P[2][3][] * (1 * ((1 - step_x) ** 0) * step_x ** 3) * (3 * ((1 - step_y) ** 1) * step_y ** 2)\
    # + \
    # P[3][0][] * (1 * ((1 - step_x) ** 3) * step_x ** 0) * (1 * ((1 - step_y) ** 0) * step_y ** 3) + \
    # P[3][1][] * (3 * ((1 - step_x) ** 2) * step_x ** 1) * (1 * ((1 - step_y) ** 0) * step_y ** 3) + \
    # P[3][2][] * (3 * ((1 - step_x) ** 1) * step_x ** 2) * (1 * ((1 - step_y) ** 0) * step_y ** 3) + \
    # P[3][3][] * (1 * ((1 - step_x) ** 0) * step_x ** 3) * (1 * ((1 - step_y) ** 0) * step_y ** 3)
    cpa = control_points_array
    point = [0,0,0]

    # PYTHON IS THAT SLOW, THAT I NEED TO DO IT IN STUPID WAY...

    rev_step_x_p3 = ((1 - step_x) ** 3)
    rev_step_x_p2 = ((1 - step_x) ** 2)
    rev_step_x_p1 = ((1 - step_x))

    rev_step_y_p3 = ((1 - step_y) ** 3)
    rev_step_y_p2 = ((1 - step_y) ** 2)
    rev_step_y_p1 = ((1 - step_y))
    
    step_x_p3 = (step_x ** 3)
    step_x_p2 = (step_x ** 2)
    step_x_p1 = (step_x)

    step_y_p3 = (step_y ** 3)
    step_y_p2 = (step_y ** 2)
    step_y_p1 = (step_y)

    for i in range(0,3):
        point[i] = \
            cpa[ 0][i] * (    rev_step_x_p3            ) * (    rev_step_y_p3            ) + \
            cpa[ 1][i] * (3 * rev_step_x_p2 * step_x_p1) * (    rev_step_y_p3            ) + \
            cpa[ 2][i] * (3 * rev_step_x_p1 * step_x_p2) * (    rev_step_y_p3            ) + \
            cpa[ 3][i] * (                    step_x_p3) * (    rev_step_y_p3            ) + \
            cpa[ 4][i] * (    rev_step_x_p3            ) * (3 * rev_step_y_p2 * step_y_p1) + \
            cpa[ 5][i] * (3 * rev_step_x_p2 * step_x_p1) * (3 * rev_step_y_p2 * step_y_p1) + \
            cpa[ 6][i] * (3 * rev_step_x_p1 * step_x_p2) * (3 * rev_step_y_p2 * step_y_p1) + \
            cpa[ 7][i] * (                    step_x_p3) * (3 * rev_step_y_p2 * step_y_p1) + \
            cpa[ 8][i] * (    rev_step_x_p3            ) * (3 * rev_step_y_p1 * step_y_p2) + \
            cpa[ 9][i] * (3 * rev_step_x_p2 * step_x_p1) * (3 * rev_step_y_p1 * step_y_p2) + \
            cpa[10][i] * (3 * rev_step_x_p1 * step_x_p2) * (3 * rev_step_y_p1 * step_y_p2) + \
            cpa[11][i] * (                    step_x_p3) * (3 * rev_step_y_p1 * step_y_p2) + \
            cpa[12][i] * (    rev_step_x_p3            ) * (                    step_y_p3) + \
            cpa[13][i] * (3 * rev_step_x_p2 * step_x_p1) * (                    step_y_p3) + \
            cpa[14][i] * (3 * rev_step_x_p1 * step_x_p2) * (                    step_y_p3) + \
            cpa[15][i] * (                    step_x_p3) * (                    step_y_p3)
    return point



# function from https://github.com/IvanNik17/python-3d-analysis-libraries.git
# Function to calculate KD-tree, closest neighbourhoods and edges between the points 
def compute_mesh_net(metric, num_neighbours=4):
    # Calculate the KD-tree of the selected feature space
    tree = KDTree(metric)
    # Query the neighbourhoods for each point of the selected feature space to each point
    d_kdtree, idx = tree.query(metric, k=num_neighbours)
    # Remove the first point in the neighbourhood as this is just the queried point itself
    idx = idx[:, 1:]

    # Create the edges array between all the points and their closest neighbours
    point_numbers = np.arange(len(metric))
    # Repeat each point in the point numbers array the number of closest neighbours -> 1,2,3,4... becomes 1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4...
    point_numbers = np.repeat(point_numbers, num_neighbours - 1)
    # Flatten  the neighbour indices array -> from [1,3,10,14], [4,7,17,23], ... becomes [1,3,10,4,7,17,23,...]
    idx_flatten = idx.flatten()
    # Create the edges array by combining the two other ones as a vertical stack and transposing them to get the input that LineSet requires
    edges = np.vstack((point_numbers, idx_flatten)).T

    return edges

def drawBezierSurface(vis, control_points_array):
    # surface dots container
    dots_points_array = []

    # compute surface dots
    for step_x in np.arange(0, 1, 0.02):
        for step_y in np.arange(0, 1, 0.02):
            step_point = computePointFunction(control_points_array, step_x, step_y)
            dots_points_array.append(step_point)


    # surface mesh filament (using not my function)
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(dots_points_array)
    line_set.lines = o3d.utility.Vector2iVector(compute_mesh_net(dots_points_array, num_neighbours=5))
    line_set.paint_uniform_color([0.25, 0.25, 0.25])
    vis.add_geometry(line_set)

    # # surface dots filament
    # point_cloud = o3d.geometry.PointCloud()
    # point_cloud.points = o3d.utility.Vector3dVector(dots_points_array)
    # # point_cloud.paint_uniform_color([1, 0.25, 0.25])
    # point_cloud.paint_uniform_color([0.25, 0.25, 0.25])
    # vis.add_geometry(point_cloud)


def drawControlPoints(vis,control_points_arra):
    for point in control_points_array:
        sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.01, resolution=10)
        sphere_mesh.compute_vertex_normals()
        sphere_mesh.paint_uniform_color([1, 0.25, 0.25])
        sphere_mesh.translate((point[0], point[1], point[2]))
        vis.add_geometry(sphere_mesh)


def drawDirectionArrows(vis):
    # arrows pointing directions
    x_cord = o3d.geometry.TriangleMesh.create_box(width=1,height=0.1,depth=0.1)
    x_cord.paint_uniform_color([1, 0.25, 0.25])
    x_cord.compute_vertex_normals()
    vis.add_geometry(x_cord)

    y_cord = o3d.geometry.TriangleMesh.create_box(width=0.1, height=1, depth=0.1)
    y_cord.paint_uniform_color([0.25, 1, 0.25])
    y_cord.compute_vertex_normals()
    vis.add_geometry(y_cord)

    z_cord = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=1)
    z_cord.paint_uniform_color([0.25, 0.25, 1])
    z_cord.compute_vertex_normals()
    vis.add_geometry(z_cord)


def loadPointsFromFile(file_name, transform_position, swich_direction = True):
    with open(file_name, "r") as file:
        file_content = file.readlines()

    line_count = -1
    single_surface_array_points = []
    object_surfaces_array = []
    for line in file_content:
        if line_count == -1 or line_count % 17 == 0:
            # pass "32" text and "3 3"
            # print(line,end='')
            if line_count % 17 == 0 and line_count > 1:
                # split to surfaces
                object_surfaces_array.append(single_surface_array_points)
                single_surface_array_points = []
        else:
            # save those points
            x, y, z = map(float, line.split())
            if swich_direction:
                single_surface_array_points.append(
                    [x + transform_position[0], z + transform_position[1], y + transform_position[2]])
            else:
                single_surface_array_points.append(
                    [x + transform_position[0], y + transform_position[1], z + transform_position[2]])

        line_count += 1
    object_surfaces_array.append(single_surface_array_points)
    return object_surfaces_array


def main():
    # Cezary Androsiuk

    teapot_points = loadPointsFromFile("teapot_points.txt", [3, 0, 3])
    teaspoon_points = loadPointsFromFile("teaspoon_points.txt", [6, 0, 3])
    teacup_points = loadPointsFromFile("teacup_points.txt", [9, 0, 3], False)

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name='Bezier Surface', width=1280, height=720)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.1171, 0.1171, 0.1171])


    # first_surface_points = [
    #     [1, 2, 1], [2, 2, 1], [3, 2, 1], [4, 4, 1],
    #     [1, 3, 2], [2, 2, 2], [3, 2, 2], [4, 2, 2],
    #     [1, 1, 3], [2, 1, 3], [3, 1, 3], [4, 1, 3],
    #     [1, 1, 4], [2, 1, 4], [3, 1, 4], [4, 3, 4]
    # ]
    # drawBezierSurface(vis, first_surface_points)

    for single_surface in teapot_points:
        drawBezierSurface(vis, single_surface)
    for single_surface in teaspoon_points:
        drawBezierSurface(vis, single_surface)
    for single_surface in teacup_points:
        drawBezierSurface(vis, single_surface)

    drawDirectionArrows(vis)

    vis.run()
    vis.destroy_window()


if __name__ == '__main__':
    main()
