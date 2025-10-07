# Examples
add_executable(basic_usage basic_usage.cpp)
target_link_libraries(basic_usage PRIVATE octree)

add_executable(spatial_queries spatial_queries.cpp)
target_link_libraries(spatial_queries PRIVATE octree)

add_executable(custom_data custom_data.cpp)
target_link_libraries(custom_data PRIVATE octree)

add_executable(raycast_example raycast_example.cpp)
target_link_libraries(raycast_example PRIVATE octree)

add_executable(voxel_pathfinding voxel_pathfinding.cpp)
target_link_libraries(voxel_pathfinding PRIVATE octree)

add_executable(frustum_culling frustum_culling.cpp)
target_link_libraries(frustum_culling PRIVATE octree)
