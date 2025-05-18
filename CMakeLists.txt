cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Importa o Pico SDK
include(pico_sdk_import.cmake)

# Configura o caminho do FreeRTOS 
set(FREERTOS_KERNEL_PATH "C:/FreeRTOS-Kernel-main")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Define o nome do projeto
project(Sistema_de_Alerta_de_Enchente C CXX ASM)

# Inicializa o Pico SDK
pico_sdk_init()

# Inclui o diretório 'lib' para as bibliotecas como ssd1306
include_directories(${CMAKE_SOURCE_DIR}/lib)

# Adiciona os arquivos fonte
add_executable(${PROJECT_NAME}  
    Sistema_de_Alerta_de_Enchente.c 
    lib/ssd1306.c
    ws2812.pio
)

# Inclui o diretório raiz do projeto para cabeçalhos
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR})

# Vincula as bibliotecas necessárias 
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib 
    hardware_gpio
    hardware_i2c  # Necessário para I2C
    hardware_adc  # Necessário para o joystick
    hardware_pio  # Necessário para a matriz WS2812
    hardware_pwm  # Necessário para o buzzer com PWM
    FreeRTOS-Kernel 
    FreeRTOS-Kernel-Heap4
)

# Habilita USB e desabilita UART para saída de depuração
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Gera arquivos adicionais (.uf2, etc.)
pico_add_extra_outputs(${PROJECT_NAME})

# Adiciona o programa PIO para a matriz WS2812
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/ws2812.pio)