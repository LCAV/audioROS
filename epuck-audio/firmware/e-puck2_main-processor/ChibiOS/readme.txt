*****************************************************************************
*** Files Organization                                                    ***
*****************************************************************************

--{root}                  - ChibiOS/RT directory.
  +--readme.txt           - This file.
  +--documentation.html   - Shortcut to the web documentation page.
  +--license.txt          - GPL license text.
  +--exception.txt        - GPL exception text (stable releases only).
  +--demos/               - Demo projects, one directory per platform.
  +--docs/                - Documentation.
  |  +--common/           - Documentation common build resources.
  |  +--hal/              - Builders for HAL.
  |  |  +--Doxyfile_*     - Doxygen project files (required for rebuild).
  |  |  +--html/          - Local HTML documentation (after rebuild).
  |  |  +--reports/       - Test reports.
  |  |  +--rsc/           - Documentation resource files (required for rebuild).
  |  |  +--src/           - Documentation source files (required for rebuild).
  |  |  +--Doxyfile_*     - Doxygen project files (required for rebuild).
  |  |  +--index.html     - Local documentation access (after rebuild).
  |  +--nil/              - Builders for NIL.
  |  |  +--Doxyfile_*     - Doxygen project files (required for rebuild).
  |  |  +--html/          - Local HTML documentation (after rebuild).
  |  |  +--reports/       - Test reports.
  |  |  +--rsc/           - Documentation resource files (required for rebuild).
  |  |  +--src/           - Documentation source files (required for rebuild).
  |  |  +--Doxyfile_*     - Doxygen project files (required for rebuild).
  |  |  +--index.html     - Local documentation access (after rebuild).
  |  +--rt/               - Builders for RT.
  |  |  +--html/          - Local HTML documentation (after rebuild).
  |  |  +--reports/       - Test reports.
  |  |  +--rsc/           - Documentation resource files (required for rebuild).
  |  |  +--src/           - Documentation source files (required for rebuild).
  |  |  +--Doxyfile_*     - Doxygen project files (required for rebuild).
  |  |  +--index.html     - Local documentation access (after rebuild).
  +--ext/                 - External libraries, not part of ChibiOS/RT.
  +--os/                  - ChibiOS components.
  |  +--hal/              - HAL component.
  |  |  +--boards/        - HAL board support files.
  |  |  +--dox/           - HAL documentation resources.
  |  |  +--include/       - HAL high level headers.
  |  |  +--lib/           - HAL libraries.
  |  |  +--osal/          - HAL OSAL implementations.
  |  |  +--src/           - HAL high level source.
  |  |  +--ports/         - HAL ports.
  |  |  +--templates/     - HAL driver template files.
  |  |     +--osal/       - HAL OSAL templates.
  |  +--nil/              - NIL RTOS component.
  |  |  +--dox/           - NIL documentation resources.
  |  |  +--include/       - NIL high level headers.
  |  |  +--src/           - NIL high level source.
  |  |  +--ports/         - NIL ports.
  |  |  +--templates/     - NIL port template files.
  |  +--rt/               - RT RTOS component.
  |  |  +--dox/           - RT documentation resources.
  |  |  +--include/       - RT high level headers.
  |  |  +--src/           - RT high level source.
  |  |  +--ports/         - RT ports.
  |  |  +--templates/     - RT port template files.
  |  +--various/          - Various portable support files.
  +--test/                - Kernel test suite source code.
  |  +--lib/              - Portable test engine.
  |  +--hal/              - HAL test suites.
  |  |  +--testbuild/     - HAL uild test and MISRA check.
  |  +--nil/              - NIL test suites.
  |  |  +--testbuild/     - NIL nuild test and MISRA check.
  |  +--rt/               - RT test suites.
  |  |  +--testbuild/     - RT build test and MISRA check.
  |  |  +--coverage/      - RT code coverage project.
  +--testhal/             - HAL integration test demos.

*****************************************************************************
*** Releases and Change Log                                               ***
*****************************************************************************

*** 3.0.1 ***
- HAL: Fixed CEC clock cannot be disabled on STM32F0xx (bug #628).
- VAR: Fixed lwIP arch code breaks with a 16-bit systick timer (bug #627).
- HAL: Fixed broken MAC driver for STM32F107 (bug #626).
- NIL: Fixed missing configuration options from NIL PPC port (bug #625).
- HAL: Fixed wrong offset in STM32 DAC driver (bug #624).
- HAL: Fixed crash on STM32F030x4/6 devices (bug #623).
- HAL: Fixed duplicated doxygen tag in STM32F4xx hal_lld.h file (bug #621).
- HAL: Fixed STM32F042 registry error (bug #620).
- HAL: Fixed wrong check in canReceive() (bug #619).
- HAL: Fixed wrong EXTI[18] vector number on STM32F373 (bug #618).
- HAL: Fixed wrong check on STM32_LSE_ENABLED definition in STM32L1xx HAL port
       (bug #617).

*** 3.0.0 ***
- NEW: Added an initialization function to the lwIP bindings, now it is
       sufficient to call lwipInit(NULL); in order to start the subsystem.
       Demo updated.
- RT:  Fixed compilation error in RT when registry is disabled (bug #614).
- NIL: Fixed OSAL_ST_MODE not defined in AVR port (bug #613).
- NIL: Fixed nilrtos redefinition of systime_t (bug #611).
- HAL: Fixed TIM2 wrongly classified as 32bits in STM32F1xx devices
       (bug #610).

*** 3.0.0p6 ***
- HAL: Removed call to localtime_r() function for non-GNU compilers in
       STM32F1xx RTC driver.
- DEM: Fixed the FatFS demo timeout, now it is expressed in milliseconds.
- DEM: Added -Wundef to all the demos and test programs in order to find
       common error cases.
- NIL: Added INTC priorities check to the e200z port.
- RT:  Added INTC priorities check to the e200z port.
- HAL: Added support for CAN in STM32F042/72 devices.
- HAL: Added support for extra DMA channels in STM32F072 devices.
- HAL: Modified the STM32 CAN driver to support unified IRQs.
- RT:  SPE-related issue in e200z ports (bug #607).
- NIL: SPE-related issue in e200z ports (bug #607).
- HAL: Fixed dependency between STM32 MAC driver and RT (bug #606).
- HAL: Fixed wrong macro names in STM32F0xx HAL driver (bug #605).
- HAL: Fixed wrong check on ADC3 in STM32F3xx ADC driver (bug #604).
- HAL: Fixed wrong macro names in STM32F3xx HAL driver (bug #603).
- HAL: Fixed errors in STM32 OTGv1 driver (bug #601).
- DEM: Fixed missing paths in e200z demos (bug #600).
- HAL: Fixed error in platform_f105_f107.mk file (bug #599).
- HAL: Fixed issue in DMA drivers when channels share ISRs (bug #597).

*** 3.0.0p5 ***
- HAL: Added no-DMA mode to the STM32 I2Cv2 driver.
- HAL: Added DAC support to all STM32 sub-platforms, added another demo for
       the STM32F3xx.
- HAL: Fixed STM32 USARTv1: incorrect txend2_cb callback behavior (bug #596).
- DEM: Fixed wrong comment in ARMCM4-STM32F401RE-NUCLEO demo (bug #595).
- HAL: Fixed STM32 SDC LLD driver initialization with Asserts disabled
       (bug #594).

*** 3.0.0p4 ***
- NEW: Added no-DMA mode to STM32 I2Cv2 driver.
- BLD: New "smart build" mode added to makefiles, now only used files are
       compiled.
- HAL: Change to the Serial_USB driver, now the INT endpoint is no more
       mandatory.
- HAL: New DAC driver implementation for STM32F4xx.
- HAL: Fixed SDC STM32 driver broken in 50MHz mode (bug #592).
- HAL: Fixed STM32 RTC SSR Register Counts Down (bug #591).
- HAL: Fixed STM32 RTC PRER Register not being set in init (bug #590).
- HAL: Fixed STM32F334 does not have an EXT18 interrupt (bug #588).
- HAL: Fixed STM32L1xx USB is missing disconnect/connect macros (bug #587).
- HAL: Fixed wrong vector number for STM32L1xx USB (bug #586).
- HAL: Fixed spurious TC interrupt in STM32 UART (v1 and v2) driver (bug #584).
- HAL: Fixed invalid checks on STM32L1xx LSI and LSE clocks (bug #583).
- HAL: Fixed RCC CAN2 macros missing in STM32F1xx platform (bug #582).
- HAL: Fixed STM32 I2Cv2 driver issue (bug 581).
- BLD: Fixed ules.mk: adding "PRE_MAKE_ALL_RULE_HOOK" (bug #580).
- BLD: Fixed rules.mk should not explicitly depend on $(BUILDDIR) (bug #579).

*** 3.0.0p3 ***
- RT:  Fixed tickless mode instability in RT (bug 577).

*** 3.0.0p2 ***
- HAL: Fixed instances of RT API in HAL drivers (bug 574).
- RT:  Fixed system time overflow issue in tickless mode (bug 573).
- RT:  Improvements to the IRQ_STORM applications.

*** 3.0.0p1 ***
- First 3.0.0 release, see release note 3.0.0.
