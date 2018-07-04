set(SRC pcd_viewer.cc
        vis.cc)

add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${GFLAGS_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
	DESTINATION ${PREFIX_DIR}/bin)
