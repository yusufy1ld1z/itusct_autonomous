# Set source files
set(SOURCES
    interface.cpp
    menu.cpp
    user.cpp
)

# Create library
add_library(menuBotLib ${SOURCES})

# Include directories
target_include_directories(menuBotLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)

# Main executable
add_executable(main main.cpp)

# Link against library
target_link_libraries(main menuBotLib)