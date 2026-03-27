idf_component_register(SRCS "esp32-common/crypt.cpp"
							"esp32-common/exception.cpp"
							"esp32-common/packet.cpp"
                       INCLUDE_DIRS "esp32-common"
					   REQUIRES main)
