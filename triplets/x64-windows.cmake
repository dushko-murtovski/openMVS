set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)
set(VCPKG_PLATFORM_TOOLSET "v143")

# zlib is linked statically so no zlib1.dll ships. DynamixMapper loads GDAL 2.4.4,
# whose zlib1.dll predates zlib 1.2.4; Windows reuses an already-loaded DLL by
# name, so a shared zlib1.dll here would resolve to GDAL's copy and libpng16.dll
# would fail to load (error 127, missing inflateReset2).
if(PORT STREQUAL "zlib")
    set(VCPKG_LIBRARY_LINKAGE static)
endif()
