add_custom_command(OUTPUT ${LIBRARY_OUTPUT_PATH}/JacoAPIWrapper.dll
    COMMAND gmcs 
        -lib:${LIBRARY_OUTPUT_PATH}/
        -r:Kinova.API.Jaco,Kinova.DLL.Data,Kinova.DLL.SafeGate 
        -target:library
        -unsafe
        -out:${LIBRARY_OUTPUT_PATH}/JacoAPIWrapper.dll
        ${CMAKE_CURRENT_SOURCE_DIR}/JacoAPIWrapper.cs 
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/JacoAPIWrapper.cs
)

add_custom_target(JacoAPIWrapper 
    DEPENDS ${LIBRARY_OUTPUT_PATH}/JacoAPIWrapper.dll
)
