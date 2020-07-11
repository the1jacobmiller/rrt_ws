cmake_minimum_required(VERSION 2.8.3)
project(lidar_apollo_cnn_seg_detect)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        #autoware_build_flags
        roscpp
        tf
        tf_conversions
        pcl_ros
        sensor_msgs
        geometry_msgs
        autoware_msgs
        )

find_package(CUDA)
#find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        tf
        tf_conversions
        pcl_ros
        sensor_msgs
        geometry_msgs
        autoware_msgs
)

###CAFFE
set(CAFFE_PATH "$ENV{HOME}/caffe/distribute")

#AW_CHECK_CUDA()

if (EXISTS "${CAFFE_PATH}")
    include_directories(${CUDA_INCLUDE_DIRS})

    IF ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^arm")
        link_directories(/usr/lib/arm-linux-gnueabihf/tegra)
    endif ()

    include_directories(
            include
            apollo
            ${catkin_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
            ${CAFFE_PATH}/include
    )

    add_executable(lidar_apollo_cnn_seg_detect
            nodes/lidar_apollo_cnn_seg_detect_node.cpp
            nodes/cnn_segmentation.cpp
            nodes/cluster2d.cpp
            nodes/feature_generator.cpp

            ## Apollo Classes
            apollo/modules/perception/lidar/lib/object_builder/object_builder.cc
            apollo/modules/perception/base/object.cc
            apollo/modules/perception/common/graph/connected_component_analysis.cc
            apollo/modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_engine.cc
            #apollo/modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.cc
            apollo/modules/perception/lidar/common/lidar_object_util.cc
            apollo/modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_track_object_matcher.cc
            apollo/modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_tracker.cc
            apollo/modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_track_object_distance.cc
            apollo/modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_shape_filter.cc
            apollo/modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_motion_filter.cc
            apollo/modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_motion_refiner.cc
            apollo/modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_motion_measurement.cc
            apollo/modules/perception/lidar/lib/tracker/common/tracked_object.cc
            apollo/modules/perception/lidar/lib/tracker/common/mlf_track_data.cc
            apollo/modules/perception/lidar/lib/tracker/common/track_data.cc
            apollo/modules/perception/lidar/lib/tracker/association/multi_hm_bipartite_graph_matcher.cc
            apollo/modules/perception/lidar/lib/tracker/association/gnn_bipartite_graph_matcher.cc
            apollo/modules/perception/lidar/lib/tracker/association/distance_collection.cc
            apollo/modules/perception/lidar/lib/tracker/measurement/measurement_collection.cc
            )
    target_link_libraries(lidar_apollo_cnn_seg_detect
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${CAFFE_PATH}/lib/libcaffe.so
            glog
            )
    add_dependencies(lidar_apollo_cnn_seg_detect
            ${catkin_EXPORTED_TARGETS}
            )

    install(TARGETS
            lidar_apollo_cnn_seg_detect
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            PATTERN ".svn" EXCLUDE)
else ()
    message("'Caffe' is not installed. 'lidar_apollo_cnn_seg_detect' will not be built.")
endif ()
