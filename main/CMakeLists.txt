set(COMPONENT_SRCS "main.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_REQUIRES
    fft
    speech
    led_strip    
    lcd
    )
register_component()
