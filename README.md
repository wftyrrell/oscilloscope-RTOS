# oscilloscope-RTOS

This project ports an oscilloscope using the Texas Instruments TM4C1294NCPDT microcontroller and configures it to be run using a TI-RTOS real-time operating system.

## Project Structure

. ├── .ccsproject ├── .cproject ├── .gitignore ├── .launches/ ├── .project ├── .settings/ ├── .vscode/ ├── .xdchelp ├── buttons.c ├── buttons.h ├── Crystalfontz128x128_ST7735.c ├── Crystalfontz128x128_ST7735.h ├── Debug/ ├── EK_TM4C1294XL.cmd ├── HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.c ├── HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.h ├── kiss_fft.c ├── kiss_fft.h ├── kissfft.hh ├── main.c ├── makefile.defs ├── README.md ├── rtos.cfg ├── sampling.c ├── sampling.h ├── src/ ├── sysctl_pll.c ├── sysctl_pll.h ├── targetConfigs/ └── tm4c1294ncpdt.cmd


## Getting Started

### Prerequisites

- [Code Composer Studio (CCS)](https://www.ti.com/tool/CCSTUDIO)
- [TI-RTOS](https://www.ti.com/tool/TIRTOS)
- [TivaWare](https://www.ti.com/tool/SW-TM4C)

### Building the Project

1. Open Code Composer Studio.
2. Import the project into your workspace.
3. Build the project by clicking on the hammer icon or by selecting `Project -> Build Project`.

### Running the Project

1. Connect the TM4C1294NCPDT microcontroller to your computer.
2. Load the program onto the microcontroller by clicking on the bug icon or by selecting `Run -> Debug`.
3. Run the program by clicking on the play icon or by selecting `Run -> Resume`.

## Files Description

- `main.c`: The main entry point of the application.
- `buttons.c` and `buttons.h`: Code for handling button inputs.
- `Crystalfontz128x128_ST7735.c` and `Crystalfontz128x128_ST7735.h`: Driver for the Crystalfontz128x128 display.
- `HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.c` and `HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.h`: Hardware abstraction layer for the display.
- `kiss_fft.c`, `kiss_fft.h`, and `kissfft.hh`: FFT library.
- `sampling.c` and `sampling.h`: Code for sampling data.
- `sysctl_pll.c` and `sysctl_pll.h`: System control and PLL configuration.
- `rtos.cfg`: Configuration file for TI-RTOS.
- `makefile.defs`: Definitions for the makefile.
- `tm4c1294ncpdt.cmd` and `EK_TM4C1294XL.cmd`: Linker command files.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
