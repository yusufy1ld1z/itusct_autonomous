# Test executables
add_executable(testJSON testJSON.cpp)
add_executable(testMenu testMenu.cpp)

# Link against library
target_link_libraries(testJSON menuBotLib)
target_link_libraries(testMenu menuBotLib)

# Include directories
target_include_directories(testJSON PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_include_directories(testMenu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
