MESSAGE( STATUS "    Configuring matt" )

include_directories(${PROJECT_BINARY_DIR}
)

set( Srcs    
	matt.cpp
)

add_executable( gotodemo ${Srcs} )

set_source_files_properties( ${Srcs} PROPERTIES COMPILE_FLAGS "${PLAYER_CFLAGS}")

set_target_properties( gotodemo PROPERTIES OUTPUT_NAME matt)

target_link_libraries( gotodemo stagewrapper 
				${PLAYER_LDFLAGS} )
set_target_properties( gotodemo PROPERTIES
             LINK_FLAGS "${PLAYER_LDFLAGS}"	  
)
