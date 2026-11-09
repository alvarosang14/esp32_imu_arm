idf_component_register(
    SRCS "bt_wrapper.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES bt nvs_flash
)
