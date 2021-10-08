project(common_math)

set(common_math_dir ${PROJECT_SOURCE_DIR}/../../..)


#----------- common_math_vec2d------------------
add_library(common_math_vec2d vec2d.cc)
target_include_directories(common_math_vec2d PUBLIC ${common_math_dir})
target_link_libraries(common_math_vec2d PUBLIC 
    cyber_common_log 
    absl::strings
)

#------------common_math_util ----------------
add_library(common_math_util math_utils.cc math_utils.h)
target_link_libraries(common_math_util PUBLIC 
    common_math_vec2d 
    Eigen3::Eigen)

#-----------common_math_geometry----------------
set(common_math_geometry_src 
    aabox2d.cc
    box2d.cc
    line_segment2d.cc 
    polygon2d.cc
)
set(common_math_geometry_headers 
    aabox2d.h 
    aaboxkdtree2d.h
    box2d.h
    line_segment2d.h
    polygon2d.h
)

add_library(common_math_geometry ${common_math_geometry_src} ${common_math_geometry_headers})
target_link_libraries(common_math_geometry 
PUBLIC 
    common_math_util
    cyber_common_log 
    common_util_string_util 
    absl::strings
    osqp::osqp
)

#-----------sin table----------------
add_library(common_math_sin_table sin_table.cc )
target_include_directories(common_math_sin_table PUBLIC ${common_math_dir})


#----------- angle ----------------
add_library(common_math_angle angle.cc )
target_include_directories(common_math_angle PUBLIC ${common_math_dir})
target_link_libraries(common_math_angle 
PUBLIC 
    common_math_sin_table
)

#_----------euler angle zxy ----------
add_library(common_math_euler_angle_zxy INTERFACE)
target_include_directories(common_math_euler_angle_zxy INTERFACE ${common_math_dir})
target_link_libraries(common_math_euler_angle_zxy
INTERFACE 
    common_math_geometry
    Eigen3::Eigen
)

#----------- quaternion--------------
add_library(common_math_quaternion INTERFACE)
target_include_directories(common_math_quaternion INTERFACE ${common_math_dir})
target_link_libraries(common_math_quaternion
INTERFACE 
    common_math_euler_angle_zxy
    common_math_geometry
    common_geometry_proto
    Eigen3::Eigen
)

#----------- matrix operation --------------
add_library(common_math_matrix_operation matrix_operations.cc)
target_include_directories(common_math_matrix_operation PUBLIC ${common_math_dir})
target_link_libraries(common_math_matrix_operation
PUBLIC 
    cyber_common_log
    Eigen3::Eigen
)

add_library(common_math_curve_fitting INTERFACE)
target_include_directories(common_math_curve_fitting INTERFACE ${common_math_dir})
target_link_libraries(common_math_curve_fitting
INTERFACE 
    common_math_matrix_operation
    Eigen3::Eigen
)

add_library(common_math_kalman_filter INTERFACE)
target_include_directories(common_math_kalman_filter INTERFACE ${common_math_dir})
target_link_libraries(common_math_kalman_filter
INTERFACE 
    cyber_common_log
    common_math_matrix_operation
    Eigen3::Eigen
)

add_library(common_math_factorial INTERFACE)
target_include_directories(common_math_factorial INTERFACE ${common_math_dir})

add_library(common_math_search search.cc)
target_include_directories(common_math_search PUBLIC ${common_math_dir})

add_library(common_math_linear_interpolation linear_interpolation.cc)
target_include_directories(common_math_linear_interpolation PUBLIC ${common_math_dir})
target_link_libraries(common_math_linear_interpolation
PUBLIC 
    # common_proto
    common_pnc_point_proto
    common_math_geometry
)

add_library(common_math_path_matcher path_matcher.cc)
target_include_directories(common_math_path_matcher PUBLIC ${common_math_dir})
target_link_libraries(common_math_path_matcher
PUBLIC 
    common_pnc_point_proto
    common_math_linear_interpolation
)


add_library(common_math_mpc_osqp mpc_osqp.cc)
target_include_directories(common_math_mpc_osqp PUBLIC ${common_math_dir})
target_link_libraries(common_math_mpc_osqp
PUBLIC 
    Eigen3::Eigen 
    cyber_common_log
    osqp::osqp
)

add_library(common_math_integral integral.cc)
target_include_directories(common_math_integral PUBLIC ${common_math_dir})
target_link_libraries(common_math_integral 
PUBLIC 
    cyber_common_log
)


add_library(common_math_lqr linear_quadratic_regulator.cc)
target_include_directories(common_math_lqr PUBLIC ${common_math_dir})
target_link_libraries(common_math_lqr 
PUBLIC 
    cyber_common_log
    Eigen3::Eigen
)


add_library(common_math_cartesian_frenet_conversion cartesian_frenet_conversion.cc)
target_include_directories(common_math_cartesian_frenet_conversion PUBLIC ${common_math_dir})
target_link_libraries(common_math_cartesian_frenet_conversion
PUBLIC 
    common_math_geometry
    cyber_common_log
    Eigen3::Eigen
)

add_library(common_math_hermite_spline INTERFACE)
target_include_directories(common_math_hermite_spline INTERFACE ${common_math_dir})
target_link_libraries(common_math_hermite_spline INTERFACE cyber_commmon_log)

add_library(common_math_math_utils math_utils.cc)
target_include_directories(common_math_math_utils PUBLIC ${common_math_dir})
target_link_libraries(common_math_math_utils
PUBLIC 
    common_math_vec2d
    Eigen3::Eigen
)

add_library(common_math INTERFACE)
target_link_libraries(common_math
INTERFACE 
    common_math_angle
    common_math_cartesian_frenet_conversion
    common_math_curve_fitting
    common_math_euler_angle_zxy
    common_math_factorial
    common_math_geometry
    common_math_integral
    common_math_kalman_filter
    common_math_linear_interpolation
    common_math_lqr
    common_math_mpc_osqp
    common_math_quaternion
    common_math_search
    common_math_sin_table
)