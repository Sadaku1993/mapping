INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
ADD_EXECUTABLE(graph_slam3d
  tutorial_slam3d.cpp
)

SET_TARGET_PROPERTIES(graph_slam3d PROPERTIES OUTPUT_NAME graph_slam3d${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(graph_slam3d tutorial_slam3d_library)
