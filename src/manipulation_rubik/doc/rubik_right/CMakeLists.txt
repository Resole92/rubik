add_executable(rubik_right_move src/rubik_right_move.cpp)
target_link_libraries(rubik_right_move ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS rubik_right_move DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
