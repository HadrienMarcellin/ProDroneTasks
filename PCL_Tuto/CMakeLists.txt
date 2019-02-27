cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_tuto)

find_package(PCL 1.5 REQUIRED)

# set(CMAKE_BUILD_TYPE Release) // Uncomment for faster runtime for complex machine learning stuff
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Basic
add_executable (matrix_transform src/Basic/matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})

# I/O
add_executable (pcd_read src/IO/pcd_read.cpp)
target_link_libraries (pcd_read ${PCL_LIBRARIES})

add_executable (pcd_write src/IO/pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

add_executable (concatenate_clouds src/IO/concatenate_clouds.cpp)
target_link_libraries (concatenate_clouds ${PCL_LIBRARIES})

# Filtering
add_executable (voxel_grid src/Filtering/voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})

add_executable (passthrough src/Filtering/passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})

add_executable (statistical_removal src/Filtering/statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

add_executable (project_inliers src/Filtering/project_inliers.cpp)
target_link_libraries (project_inliers ${PCL_LIBRARIES})

add_executable (extract_indices src/Filtering/extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})

add_executable (remove_outliers src/Filtering/remove_outliers.cpp)
target_link_libraries (remove_outliers ${PCL_LIBRARIES})

# Features
add_executable (normal_estimation_using_internal_images src/Features/normal_estimation_using_internal_images.cpp)
target_link_libraries (normal_estimation_using_internal_images ${PCL_LIBRARIES})

add_executable (narf_feature_extraction src/Features/narf_feature_extraction.cpp)
target_link_libraries (narf_feature_extraction ${PCL_LIBRARIES})

add_executable (moment_of_inertia src/Features/moment_of_inertia.cpp)
target_link_libraries (moment_of_inertia ${PCL_LIBRARIES})

# Keypoints
add_executable (narf_keypoint src/Keypoints/narf_keypoint.cpp)
target_link_libraries (narf_keypoint ${PCL_LIBRARIES})

# KdTree
add_executable (kd_tree src/KdTree/kd_tree.cpp)
target_link_libraries (kd_tree ${PCL_LIBRARIES})

# Octree
add_executable (octree_search src/Octree/octree_search.cpp)
target_link_libraries (octree_search ${PCL_LIBRARIES})

add_executable (octree_change_detection src/Octree/octree_change_detection.cpp)
target_link_libraries (octree_change_detection ${PCL_LIBRARIES})

add_executable (octree_change_detection_bun0 src/Octree/octree_change_detection_bun0.cpp)
target_link_libraries (octree_change_detection_bun0 ${PCL_LIBRARIES})


# Range Images
add_executable (range_image_creation src/Range_Images/range_image_creation.cpp)
target_link_libraries (range_image_creation ${PCL_LIBRARIES})

add_executable (range_border_extraction src/Range_Images/range_border_extraction.cpp)
target_link_libraries (range_border_extraction ${PCL_LIBRARIES})

# Recognition
add_executable (correspondance_grouping src/Recognition/correspondance_grouping.cpp)
target_link_libraries (correspondance_grouping ${PCL_LIBRARIES})

add_executable (implicit_shape_model src/Recognition/implicit_shape_model.cpp)
target_link_libraries (implicit_shape_model ${PCL_LIBRARIES})

# Registration
add_executable (iterative_closest_point src/Registration/iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

add_executable (incremental_register_pairs_cloud src/Registration/incremental_register_pairs_cloud.cpp)
target_link_libraries (incremental_register_pairs_cloud ${PCL_LIBRARIES})

add_executable (normal_distribution_transform src/Registration/normal_distribution_transform.cpp)
target_link_libraries (normal_distribution_transform ${PCL_LIBRARIES})

add_executable (alignement_prerejective src/Registration/alignement_prerejective.cpp)
target_link_libraries (alignement_prerejective ${PCL_LIBRARIES})

# Sample Consensus
add_executable (random_sample_consensus src/Sample_Consensus/random_sample_consensus.cpp)
target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})

# Segmentation
add_executable (planar_segmentation src/Segmentation/planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})

add_executable (cylinder_segmentation src/Segmentation/cylinder_segmentation.cpp)
target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})

add_executable (cluster_extraction src/Segmentation/cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

add_executable (region_growing src/Segmentation/region_growing.cpp)
target_link_libraries (region_growing ${PCL_LIBRARIES})

add_executable (color_based_region_growing src/Segmentation/color_based_region_growing.cpp)
target_link_libraries (color_based_region_growing ${PCL_LIBRARIES})

add_executable (min_cut_segmentation src/Segmentation/min_cut_segmentation.cpp)
target_link_libraries (min_cut_segmentation ${PCL_LIBRARIES})

add_executable (conditional_euclidean_clustering src/Segmentation/conditional_euclidean_clustering.cpp)
target_link_libraries (conditional_euclidean_clustering ${PCL_LIBRARIES})

add_executable (don_segmentation src/Segmentation/don_segmentation.cpp)
target_link_libraries (don_segmentation ${PCL_LIBRARIES})

# Surface
add_executable (resampling src/Surface/resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})

add_executable (concave_hull_2d src/Surface/concave_hull_2d.cpp)
target_link_libraries (concave_hull_2d ${PCL_LIBRARIES})

add_executable (triangulation src/Surface/triangulation.cpp)
target_link_libraries (triangulation ${PCL_LIBRARIES})

add_executable (greedy_projection src/Surface/greedy_projection.cpp)
target_link_libraries (greedy_projection ${PCL_LIBRARIES})

# Visualization
add_executable (cloud_viewer src/Visualization/cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

add_executable (range_image_visualization src/Visualization/range_image_visualization.cpp)
target_link_libraries (range_image_visualization ${PCL_LIBRARIES})

add_executable (pcl_plotter src/Visualization/pcl_plotter.cpp)
target_link_libraries (pcl_plotter ${PCL_LIBRARIES})

# Application
add_executable (template_alignment src/Applications/template_alignment.cpp)
target_link_libraries (template_alignment ${PCL_LIBRARIES})
