set(srcs
    "bo_phycp.c"
)

set(priv_requires
    nvs_flash
    spi_flash
    esp_phy
)

idf_component_register(
    SRCS "${srcs}"
    PRIV_REQUIRES "${priv_requires}"
)

if(CONFIG_BO_PHYCP_ENABLED)
target_link_libraries(${COMPONENT_TARGET} "-z muldefs")
target_link_libraries(${COMPONENT_TARGET} "-u bo_phycp_include")
endif()