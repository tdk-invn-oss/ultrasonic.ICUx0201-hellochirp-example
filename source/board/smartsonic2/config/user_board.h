/*
<!-- BEGIN INVN LICENSE -->
TDK InvenSense 5 Clause License

Copyright (c) 2025, Invensense, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

4. This software, with or without modification, must only be used with a
TDK InvenSense sensor.

5. Any software provided in binary form under this license, whether embedded
in source code or provided as a compiled library or application, must not
be reverse engineered, decompiled, modified and/or disassembled.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
<!-- END INVN LICENSE -->
*/

#ifndef _USER_BOARD_H_
#define _USER_BOARD_H_

#define BOARD_FREQ_SLCK_XTAL     (32768U)
#define BOARD_FREQ_SLCK_BYPASS   (32768U)
#define BOARD_FREQ_MAINCK_XTAL   0 /* Not Mounted */
#define BOARD_FREQ_MAINCK_BYPASS 0 /* Not Mounted */
#define BOARD_MCK                CHIP_FREQ_CPU_MAX
/*TBD startup time needs to be adjusted according to measurements */
#define BOARD_OSC_STARTUP_US     15625

#endif /* _USER_BOARD_H_ */