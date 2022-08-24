add_executable(eyes src/eyes.cpp)
target_link_libraries(eyes ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS eyes DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
