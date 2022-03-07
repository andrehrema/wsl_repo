idf_component_register(SRCS "joystick.c"
                    INCLUDE_DIRS "."
                    REQUIRES lwip esp_netif esp_event esp_wifi soc esp_hw_support driver)


set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_s_sources "ulp/ulpcode.S")
set(ulp_exp_dep_srcs "joystick.c")
ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")