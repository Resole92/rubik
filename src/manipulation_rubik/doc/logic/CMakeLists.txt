add_executable(logic src/logic.cpp)
target_link_libraries(logic ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS logic DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
