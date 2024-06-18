add_library(StereoVisionSLAM 
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp
        loopclosure.cpp
        dense_reconstruction.cpp
        )

target_link_libraries(StereoVisionSLAM ${THIRD_PARTY_LIBS})