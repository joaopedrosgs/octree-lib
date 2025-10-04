# Examples
add_executable(basic_usage basic_usage.cpp)
target_link_libraries(basic_usage PRIVATE octree)

add_executable(spatial_queries spatial_queries.cpp)
target_link_libraries(spatial_queries PRIVATE octree)

add_executable(custom_data custom_data.cpp)
target_link_libraries(custom_data PRIVATE octree)
