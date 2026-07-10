idf_component_register(
    SRCS "fsm/lsm6dsox_fsm.cpp" "ml/features.cpp" "ml/model.cpp"
    INCLUDE_DIRS "." "fsm" "ml"
    REQUIRES common drivers
)
