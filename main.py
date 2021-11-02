import pcl
import open3d as o3d
import numpy as np
from pcl import visualization

def get_filter(point_cloud, name_axis = 'z', min_axis = 0.6, max_axis = 1.1):
    pass_filter = point_cloud.make_passthrough_filter()
    pass_filter.set_filter_field_name(name_axis);
    pass_filter.set_filter_limits(min_axis, max_axis)
    return pass_filter.filter()

def get_segmentation(point_cloud, max_distance = 5):

    segmenter = point_cloud.make_segmenter()

    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_distance_threshold(max_distance)

    #obtain inlier indices and model coefficients
    inlier_indices, coefficients = segmenter.segment()

    inliers = point_cloud.extract(inlier_indices, negative = False)
    outliers = point_cloud.extract(inlier_indices, negative = True)

    return inliers, outliers

def remove_background(pointcloud, background_depth=0.2):
    # get pointcloud coordinates range
    pointcloud_array = pointcloud.to_array()
    max_array = np.max(pointcloud_array, axis=0)

    # remove surface (table/ground)
    filtered_cloud = get_filter(point_cloud=pointcloud,
                                         name_axis='z', min_axis=background_depth, max_axis=max_array[2])
    rest, table_cloud = get_segmentation(filtered_cloud)

    # remove wall
    filtered_cloud = get_filter(point_cloud=rest,
                                name_axis='y', min_axis=0, max_axis=max_array[1]-background_depth)
    objects,_ = get_segmentation(filtered_cloud)

    return objects

def visualize():
    pointCloud = o3d.io.read_point_cloud('data/segmented.pcd')
    original = o3d.io.read_point_cloud(file)
    o3d.visualization.draw_geometries([original])

    pointCloud.paint_uniform_color([0, 1, 0])
    original.paint_uniform_color([0.6,0.6,0.6])

    p_load=np.asarray(original.points)
    p_color=np.asarray(original.colors)

    p1_load=np.asarray(pointCloud.points)
    p1_color=np.asarray(pointCloud.colors)

    p_load=np.concatenate((p1_load,p_load), axis=0)
    p_color=np.concatenate((p1_color,p_color), axis=0)

    pcd=o3d.geometry.PointCloud()
    pcd.points=o3d.utility.Vector3dVector(p_load)
    pcd.colors=o3d.utility.Vector3dVector(p_color)
    visualiser = o3d.visualization.VisualizerWithVertexSelection();
    visualiser.create_window(window_name="Point picker");
    visualiser.add_geometry(pcd);
    visualiser.run();
    visualiser.destroy_window()

if __name__=='__main__':
    file='data/world1.pcd'
    cloud = pcl.load(file)
    objects = remove_background(cloud)
    pcl.save(objects, 'data/segmented.pcd')
    visualize()