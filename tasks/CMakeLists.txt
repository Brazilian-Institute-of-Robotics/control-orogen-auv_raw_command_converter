# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(auv_raw_command_converterTaskLib)
ADD_LIBRARY(${AUV_RAW_COMMAND_CONVERTER_TASKLIB_NAME} SHARED 
    ${AUV_RAW_COMMAND_CONVERTER_TASKLIB_SOURCES})
add_dependencies(${AUV_RAW_COMMAND_CONVERTER_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${AUV_RAW_COMMAND_CONVERTER_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${AUV_RAW_COMMAND_CONVERTER_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${AUV_RAW_COMMAND_CONVERTER_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${AUV_RAW_COMMAND_CONVERTER_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${AUV_RAW_COMMAND_CONVERTER_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${AUV_RAW_COMMAND_CONVERTER_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${AUV_RAW_COMMAND_CONVERTER_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${AUV_RAW_COMMAND_CONVERTER_TASKLIB_HEADERS}
    DESTINATION include/orocos/auv_raw_command_converter)

