# Licensing

## Description

The __SmartSonic example projects__, including those supporting the TDK InvenSense CHx01 series parts and the ICU-x0201 series parts, contain a mix of licenses.  
The individual source files either include the license text directly, refer to one of the licenses, or are in a directory containing the license for those source files.  
  
The __SmartSonic example projects__ are intended to be used with particular hardware, such as the SmartSonic2 board, based on a MicroChip microcontroller, or the Nucleo board, based on an STM32 microcontroller.  
This is why the licenses from these vendors are also included.  
  
You may adapt TDK InvenSense code from this project to be used with a different microcontroller and avoid the restrictions from those particular microcontroller vendors.

## Details

This project has content provided under one of following license:

* Proprietary

  * __TDK InvenSense 5 Clause License__ - [Definition](https://github.com/tdk-invn-oss/tdk_invensense_5_clause_license)
  * __Atmel ASF License__, used by source code provided by Atmel (now Microchip) and intended to be used with Atmel (or Microchip) parts only.
  * __Microchip ASF License__, used by source code provided by Microchip and intended to be used with Microchip parts only.
  * __CMSIS EULA__, provided as a pdf in the `source/board/smartsonic2/thirdparty/CMSIS` directory.

* Open Source

  * __BSD 3 Clause License "New" or "Revised" License__ - [Definition](https://spdx.org/licenses/BSD-3-Clause.html)
  * __MIT License__ - [Definition](https://spdx.org/licenses/MIT.html)


## Scope of application per license

| License                                    | Scope of application     |
|--------------------------------------------|---------|
| [TDK InvenSense 5 Clause License](LICENSE_INVN) | <ul><li>`source/algo` directory</li><li>Files in the `source/drivers/invn-soniclib` directory, except as noted by the LICENSE file in said directory.</li><li>Files in the `source/drivers/invn-soniclib-plugins` directory</li><li>Files that reference or include the "TDK InvenSense 5 Clause" license</li></ul> |
| [Atmel ASF License](LICENSE_OTHERS)<br><br>[Microchip ASF License](LICENSE_OTHERS) | <ul><li>Files in `source/boards/smartsonic*` directories, unless stated otherwise in the file.</li></ul> |
| [CMSIS EULA](LICENSE_OTHERS) | Usage of Arm's Cortex Microcontroller Software Interface Standard (CMSIS) |
| [BSD 3 Clause License "New" or "Revised" License](LICENSE_OTHERS) | <ul><li>STM32Cube library</li><li>Files that reference or include the BSD 3 Clause License "New" or "Revised" License</li></ul> |
| [MIT License](LICENSE_OTHERS) | <ul><li>Used by some third party library code, where specified in the source files.</li></ul> |
