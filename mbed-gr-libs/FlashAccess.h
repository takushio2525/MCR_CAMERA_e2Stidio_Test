/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2017 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/

#ifndef FLASH_ACCESS_H
#define FLASH_ACCESS_H

#include "mbed.h"
#include "iodefine.h"
#include "spibsc_iobitmask.h"
#include "spibsc.h"

#define _RAM_PRG __attribute__((section("RAM_CODE")))

/** FlashAccess class
 *  Flash access library for GR-Boards.
 */
class FlashAccess {
public:
    /** Constructor
     *
     */
    FlashAccess();

    /** Sector Erase
     *
     *  The Sector Erase instruction sets all memory within a specified sector (4K-bytes) 
     *  to the erased state of all 1s (FFh).
     *
     *  @param addr      sector address
     *  @return          true = success, false = failure
     */
    bool SectorErase(uint32_t addr) _RAM_PRG;

    /** Page Program
     *
     *  The Page Program instruction allows from one byte to 256 bytes (a page) of data 
     *  to be programmed at previously erased (FFh) memory locations.
     *
     *  @param addr      starting address
     *  @param buf       the buffer to read into
     *  @param size      size of the data
     *  @return          true = success, false = failure
     */
    bool PageProgram(uint32_t addr, uint8_t * buf, int32_t size) _RAM_PRG;

    /** Read
     *
     *  The Read Data instruction allows one or more data bytes to be sequentially read 
     *  from the memory.
     *
     *  @param addr      starting address
     *  @param buf       the buffer to write from
     *  @param size      size of the data
     *  @return          true = success, false = failure
     */
    bool Read(uint32_t addr, uint8_t * buf, int32_t size) _RAM_PRG;

protected:
    typedef struct {
        uint32_t cdb;       /* bit-width : command */
        uint32_t ocdb;      /* bit-width : optional command */
        uint32_t adb;       /* bit-width : address */
        uint32_t opdb;      /* bit-width : option data */
        uint32_t spidb;     /* bit-width : data */

        uint32_t cde;       /* Enable : command */
        uint32_t ocde;      /* Enable : optional command */
        uint32_t ade;       /* Enable : address */
        uint32_t opde;      /* Enable : option data */
        uint32_t spide;     /* Enable : data */

        uint32_t sslkp;     /* SPBSSL level */
        uint32_t spire;     /* Enable data read */
        uint32_t spiwe;     /* Enable data write */

        uint32_t dme;       /* Enable : dummy cycle */

        uint32_t addre;     /* DDR enable : address  */
        uint32_t opdre;     /* DDR enable : option data */
        uint32_t spidre;    /* DDR enable : data */

        uint8_t dmdb;       /* bit-width : dummy cycle */
        uint8_t dmcyc;      /* number of dummy cycles */

        uint8_t  cmd;       /* command */
        uint8_t  ocmd;      /* optional command */
        uint32_t addr;      /* address */
        uint8_t  opd[4];    /* option data 3/2/1/0 */
        uint32_t smrdr[2];  /* read data */
        uint32_t smwdr[2];  /* write data */
    } st_spibsc_spimd_reg_t;

    volatile struct st_spibsc*  SPIBSC;
    st_spibsc_spimd_reg_t spimd_reg;

    bool _SectorErase(uint32_t addr) _RAM_PRG;
    bool _PageProgram(uint32_t addr, uint8_t * buf, int32_t size) _RAM_PRG;
    bool _Read(uint32_t addr, uint8_t * buf, int32_t size) _RAM_PRG;
    bool _WriteEnable(void) _RAM_PRG;
    bool _busy_wait(void) _RAM_PRG;
    bool _read_register(uint8_t cmd, uint8_t * status) _RAM_PRG;
    bool data_send(uint32_t bit_width, uint32_t spbssl_level, uint8_t * buf, int32_t size) _RAM_PRG;
    bool data_recv(uint32_t bit_width, uint32_t spbssl_level, uint8_t * buf, int32_t size) _RAM_PRG;
    void spi_mode(void) _RAM_PRG;
    void ex_mode(void) _RAM_PRG;
    void clear_spimd_reg(st_spibsc_spimd_reg_t * regset) _RAM_PRG;
    bool spibsc_transfer(st_spibsc_spimd_reg_t * regset) _RAM_PRG;
    uint32_t RegRead_32(volatile uint32_t * ioreg, uint32_t shift, uint32_t mask) _RAM_PRG;
    void RegWwrite_32(volatile uint32_t * ioreg, uint32_t write_value, uint32_t shift, uint32_t mask) _RAM_PRG;

private:
    /*  SPI Multi-I/O bus address space address definitions */
    #define SPIBSC_ADDR_START  (0x18000000uL)
    #define SPIBSC_ADDR_END    (0x1BFFFFFFuL)

    typedef struct {
        uint32_t b0             : 1 ;       /* bit 0        : -         (0)                                   */
        uint32_t b1             : 1 ;       /* bit 1        : -         (1)                                   */
        uint32_t B              : 1 ;       /* bit 2        : B         Memory region attribute bit           */
        uint32_t C              : 1 ;       /* bit 3        : C         Memory region attribute bit           */
        uint32_t XN             : 1 ;       /* bit 4        : XN        Execute-never bit                     */
        uint32_t Domain         : 4 ;       /* bit 8-5      : Domain    Domain field                          */
        uint32_t b9             : 1 ;       /* bit 9        : IMP       IMPLEMENTATION DEFINED                */
        uint32_t AP1_0          : 2 ;       /* bit 11-10    : AP[1:0]   Access permissions bits:bit1-0        */
        uint32_t TEX            : 3 ;       /* bit 14-12    : TEX[2:0]  Memory region attribute bits          */
        uint32_t AP2            : 1 ;       /* bit 15       : AP[2]     Access permissions bits:bit2          */
        uint32_t S              : 1 ;       /* bit 16       : S         Shareable bit                         */
        uint32_t nG             : 1 ;       /* bit 17       : nG        Not global bit                        */
        uint32_t b18            : 1 ;       /* bit 18       : -         (0)                                   */
        uint32_t NS             : 1 ;       /* bit 19       : NS        Non-secure bit                        */
        uint32_t base_addr      : 12;       /* bit 31-20    : PA[31:20] PA(physical address) bits:bit31-20    */
    } mmu_ttbl_desc_section_t;

    mmu_ttbl_desc_section_t desc_tbl[(SPIBSC_ADDR_END >> 20) - (SPIBSC_ADDR_START >> 20) + 1];

    void change_mmu_ttbl_spibsc(uint32_t type) _RAM_PRG;
    void spibsc_stop(void) _RAM_PRG;
    void cache_control(void) _RAM_PRG;
};
#endif
