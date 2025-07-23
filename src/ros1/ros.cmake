message(STATUS "ROS1 activated, building ROS1 stuff")

file(GLOB_RECURSE ROS1_SOURCES src/ros1/*.cpp)

foreach (src_file IN LISTS ROS1_SOURCES)
    get_filename_component(name ${src_file} NAME_WE)
    add_executable(${name} ${src_file})
    erl_target_dependencies(${name})
    erl_collect_targets(EXECUTABLES ${name})
endforeach ()
