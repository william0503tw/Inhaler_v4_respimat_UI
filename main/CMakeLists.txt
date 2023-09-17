idf_component_register(
    SRCS "flow_transform.cc" "mpu6050.cc" "main.cc" "main_functions.cc"  "model.cc" "model_settings.cc" "audio_provider.cc" "feature_provider.cc" "buzzer.cc"
         "message_package.cc"
    REQUIRES "tflite-lib" "driver" "esp_timer" "MPU6050" "I2Cdev"
    INCLUDE_DIRS ".")

    # Reduce the level of paranoia to be able to compile sources
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-maybe-uninitialized
    -Wno-missing-field-initializers
    -Wno-error=sign-compare
    -Wno-error=double-promotion
    -Wno-type-limits)
