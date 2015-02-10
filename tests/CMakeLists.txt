


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../app/robots/iCubGenova01/yarpWholeBodyInterface.ini
                                          ${CMAKE_CURRENT_BINARY_DIR}/yarpWholeBodyInterface.ini)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../app/robots/iCubGenova01/model.urdf
                                          ${CMAKE_CURRENT_BINARY_DIR}/model.urdf)

add_executable(yarpWholeBodyModelTest yarpWholeBodyModelTest.cpp)

target_link_libraries(yarpWholeBodyModelTest yarpwholebodyinterface)

add_test(NAME test_yarpWholeBodyModel COMMAND yarpWholeBodyModelTest)
