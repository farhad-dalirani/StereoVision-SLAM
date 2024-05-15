SET(TEST_SOURCES test_frame test_feature test_dataset test_mappoint test_camera)

FOREACH (test_src ${TEST_SOURCES})
    ADD_EXECUTABLE(${test_src} ${test_src}.cpp)
    TARGET_LINK_LIBRARIES(${test_src} ${THIRD_PARTY_LIBS} StereoVisionSLAM)
    ADD_TEST(${test_src} ${test_src})
ENDFOREACH (test_src)