# Worker process for DynamixMapper (see DynamixMvsHost.cpp): the hosted apps' own sources are
# compiled in unchanged, with main() renamed so the host can call each one. The rename is a
# source property of this directory only, so the apps' own targets are not affected.
set(HOSTED_APPS DensifyPointCloud CreateStructure ReconstructMesh RefineMesh TextureMesh)
set(HOSTED_APP_SOURCES "")
foreach(app ${HOSTED_APPS})
	set(appSource "${CMAKE_CURRENT_SOURCE_DIR}/../${app}/${app}.cpp")
	set_source_files_properties("${appSource}" PROPERTIES COMPILE_DEFINITIONS "main=${app}_main")
	list(APPEND HOSTED_APP_SOURCES "${appSource}")
endforeach()
source_group("Hosted apps" FILES ${HOSTED_APP_SOURCES})

if(MSVC)
	create_rc_files(DynamixMvsHost)
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "${CMAKE_CURRENT_BINARY_DIR}/*.rc")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

# like CreateStructure, without link-time code generation (nothing hot lives in the app sources)
cxx_executable_with_flags(DynamixMvsHost "Apps" "${cxx_default}" "MVS;SFM" DISABLE_IPO ${LIBRARY_FILES_C} ${LIBRARY_FILES_H} ${HOSTED_APP_SOURCES})

# Install
INSTALL(TARGETS DynamixMvsHost
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
