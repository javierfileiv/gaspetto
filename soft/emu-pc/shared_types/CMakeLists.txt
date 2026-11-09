# shared_types - Application-specific type definitions (events, states) and the program builder
# shared between production and the bench
add_library(shared_types STATIC src/PieceProgram.cpp)
target_include_directories(shared_types PUBLIC include)
target_link_libraries(shared_types INTERFACE state_machine_framework_itf)
