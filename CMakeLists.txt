cmake_minimum_required( VERSION 2.8 )
project( CrayonRestruction )

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )



find_package( OpenCV 3.4 REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)
find_package(PCL 1.2 REQUIRED)


list (REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${G2O_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
)

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_types_sba
        g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
        g2o_stuff g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )




link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED
        src/frame.cpp
        src/mappoint.cpp
        src/tracker.cpp
        src/optimizer.cpp
        src/viewer.cpp
        src/calibration.cpp
        src/io.cpp
        )

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${Eigen_LIBS}
        ${G2O_LIBS}
        ${PCL_LIBRARIES}
        )

add_executable(camerapara src/parameter.cpp)
target_link_libraries(camerapara ${PROJECT_NAME})

add_executable( restruct src/main.cpp )
target_link_libraries( restruct ${PROJECT_NAME})

