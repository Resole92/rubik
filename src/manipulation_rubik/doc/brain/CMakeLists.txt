add_executable(brain src/brain.cpp)
target_link_libraries(brain ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS brain DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
