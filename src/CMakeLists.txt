remake_find_package(gsl CONFIG)

remake_add_directories(lib)
remake_add_directories(bin COMPONENT utils)
remake_pkg_config_generate()
