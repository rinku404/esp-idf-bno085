# Generates CONFIG_BNO085_MAX_INSTANCES independently-linked copies of the
# UNMODIFIED vendored sh2.c/shtp.c pair, plus a vtable registry TU.
# Must be include()-d only outside CMAKE_BUILD_EARLY_EXPANSION, and before
# idf_component_register(), since SRCS is resolved at configure time here
# (no GENERATED property / custom command indirection is used).

if(NOT DEFINED CONFIG_BNO085_MAX_INSTANCES)
    message(FATAL_ERROR
        "CONFIG_BNO085_MAX_INSTANCES undefined - sdkconfig.cmake not yet "
        "included in this scope. This script must only run outside the "
        "early component-requirements-expansion pass.")
endif()

if(CONFIG_BNO085_MAX_INSTANCES LESS 1)
    message(FATAL_ERROR "CONFIG_BNO085_MAX_INSTANCES must be >= 1")
endif()

set(BNO085_MAX_INSTANCES ${CONFIG_BNO085_MAX_INSTANCES})
math(EXPR BNO085_LAST_SLOT "${BNO085_MAX_INSTANCES} - 1")

get_filename_component(SH2_VENDOR_DIR "${CMAKE_CURRENT_LIST_DIR}/../sh2" ABSOLUTE)
file(TO_CMAKE_PATH "${SH2_VENDOR_DIR}" SH2_VENDOR_DIR)

set(SH2_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${SH2_GENERATED_DIR}")

set(SH2_GENERATED_SRCS "")
set(_externs "")
set(_rows "")

foreach(BNO085_SLOT RANGE 0 ${BNO085_LAST_SLOT})
    set(_out "${SH2_GENERATED_DIR}/sh2_instance_${BNO085_SLOT}.c")
    configure_file("${CMAKE_CURRENT_LIST_DIR}/sh2_instance.c.in" "${_out}" @ONLY)
    list(APPEND SH2_GENERATED_SRCS "${_out}")

    string(APPEND _externs
        "extern int  sh2_open_slot${BNO085_SLOT}(sh2_Hal_t *pHal, sh2_EventCallback_t *eventCallback, void *eventCookie);\n"
        "extern void sh2_close_slot${BNO085_SLOT}(void);\n"
        "extern void sh2_service_slot${BNO085_SLOT}(void);\n"
        "extern int  sh2_setSensorCallback_slot${BNO085_SLOT}(sh2_SensorCallback_t *callback, void *cookie);\n"
        "extern int  sh2_setSensorConfig_slot${BNO085_SLOT}(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig);\n"
    )
    string(APPEND _rows
        "    { sh2_open_slot${BNO085_SLOT}, sh2_close_slot${BNO085_SLOT}, sh2_service_slot${BNO085_SLOT}, "
        "sh2_setSensorCallback_slot${BNO085_SLOT}, sh2_setSensorConfig_slot${BNO085_SLOT} },\n"
    )
endforeach()

set(_registry "${SH2_GENERATED_DIR}/sh2_registry.c")
file(WRITE "${_registry}"
    "/* AUTO-GENERATED - DO NOT EDIT. See GenerateSh2Instances.cmake. */\n"
    "#include \"sh2_multi.h\"\n"
    "#include <stdbool.h>\n"
    "#include <stddef.h>\n\n"
)
file(APPEND "${_registry}" "${_externs}\n")
file(APPEND "${_registry}"
    "#define BNO085_SH2_SLOT_COUNT (${BNO085_MAX_INSTANCES})\n\n"
    "static const sh2_vtable_t s_vtables[BNO085_SH2_SLOT_COUNT] = {\n"
)
file(APPEND "${_registry}" "${_rows}")
file(APPEND "${_registry}"
    "};\n\n"
    "static sh2_instance_slot_t s_slots[BNO085_SH2_SLOT_COUNT];\n"
    "static bool s_slot_in_use[BNO085_SH2_SLOT_COUNT];\n"
    "static bool s_slots_init_done = false;\n\n"
    "static void s_init_slots(void)\n"
    "{\n"
    "    for (int i = 0; i < BNO085_SH2_SLOT_COUNT; i++) {\n"
    "        s_slots[i].index  = i;\n"
    "        s_slots[i].vtable = &s_vtables[i];\n"
    "        s_slot_in_use[i]  = false;\n"
    "    }\n"
    "    s_slots_init_done = true;\n"
    "}\n\n"
    "const sh2_instance_slot_t *sh2_multi_acquire_slot(void)\n"
    "{\n"
    "    if (!s_slots_init_done) { s_init_slots(); }\n"
    "    for (int i = 0; i < BNO085_SH2_SLOT_COUNT; i++) {\n"
    "        if (!s_slot_in_use[i]) {\n"
    "            s_slot_in_use[i] = true;\n"
    "            return &s_slots[i];\n"
    "        }\n"
    "    }\n"
    "    return NULL;\n"
    "}\n\n"
    "void sh2_multi_release_slot(const sh2_instance_slot_t *slot)\n"
    "{\n"
    "    if (!slot) return;\n"
    "    if ((slot->index < 0) || (slot->index >= BNO085_SH2_SLOT_COUNT)) return;\n"
    "    if (slot != &s_slots[slot->index]) return;\n"
    "    s_slot_in_use[slot->index] = false;\n"
    "}\n"
)

list(APPEND SH2_GENERATED_SRCS "${_registry}")
