set(COMPONENT_SRCS "speech_recsrc.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_REQUIRES
   spiffs
   fft
    )
register_component()
