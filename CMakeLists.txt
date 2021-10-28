
option(BUILD_RBQCONTROLLER "Building a RBQ Controller \"RBQController\"" ON)



if(BUILD_RBQCONTROLLER)
  set(target RBQController)
	add_cnoid_simple_controller(${target} SHARED RBQController.cpp)
endif()




