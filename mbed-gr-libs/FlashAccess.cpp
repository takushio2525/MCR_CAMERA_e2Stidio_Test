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

#include "mbed.h"
#include "FlashAccess.h"

/* ---- serial flash command ---- */
#define SFLASHCMD_SECTOR_ERASE       (0x20u)    /* SE     3-byte address(1bit)             */
#define SFLASHCMD_PAGE_PROGRAM       (0x02u)    /* PP     3-byte address(1bit), data(1bit) */
#define SFLASHCMD_READ               (0x03u)    /* READ   3-byte address(1bit), data(1bit) */
#define SFLASHCMD_READ_STATUS_REG    (0x05u)    /* RDSR                         data(1bit) */
#define SFLASHCMD_WRITE_ENABLE       (0x06u)    /* WREN                                    */
/* ---- serial flash register definitions ---- */
#define STREG_BUSY_BIT               (0x01u)    /* SR.[0]BUSY Erase/Write In Progress (RO) */

/* Definition of the base address for the MMU translation table */
#if defined(__CC_ARM) || defined(__GNUC__)
extern uint32_t Image$$TTB$$ZI$$Base;
#define TTB         ((uint32_t)&Image$$TTB$$ZI$$Base)   /* using linker symbol */
#elif defined(__ICCARM__)
#pragma section="TTB"
#define TTB         ((uint32_t)__section_begin("TTB"))
#endif

/** public **/

FlashAccess::FlashAccess() : SPIBSC(&SPIBSC0) {
}

bool FlashAccess::SectorErase(uint32_t addr) {
    bool ret;
#if defined (__ICCARM__)
    int was_masked = __disable_irq_iar();
#else
    int was_masked = __disable_irq();
#endif
    spi_mode();
    ret = _SectorErase(addr);
    ex_mode();
    if (0 == was_masked) {
        __enable_irq();
    }
    return ret;
}

bool FlashAccess::PageProgram(uint32_t addr, uint8_t * buf, int32_t size) {
    bool ret;
#if defined (__ICCARM__)
    int was_masked = __disable_irq_iar();
#else
    int was_masked = __disable_irq();
#endif
    spi_mode();
    ret = _PageProgram(addr, buf, size);
    ex_mode();
    if (0 == was_masked) {
        __enable_irq();
    }
    return ret;
}

bool FlashAccess::Read(uint32_t addr, uint8_t * buf, int32_t size) {
    bool ret;
#if defined (__ICCARM__)
    int was_masked = __disable_irq_iar();
#else
    int was_masked = __disable_irq();
#endif
    spi_mode();
    ret = _Read(addr, buf, size);
    ex_mode();
    if (0 == was_masked) {
        __enable_irq();
    }
    return ret;
}

/** protected **/

bool FlashAccess::_SectorErase(uint32_t addr) {
    bool ret;

    /* ---- Write enable   ---- */
    ret = _WriteEnable();      /* WREN Command */
    if (ret == false) {
        return ret;
    }

    /* ---- spimd_reg init ---- */
    clear_spimd_reg(&spimd_reg);

    /* ---- command ---- */
    spimd_reg.cde    = SPIBSC_OUTPUT_ENABLE;
    spimd_reg.cdb    = SPIBSC_1BIT;
    spimd_reg.cmd    = SFLASHCMD_SECTOR_ERASE;

    /* ---- address ---- */
    spimd_reg.ade    = SPIBSC_OUTPUT_ADDR_24;
    spimd_reg.addre  = SPIBSC_SDR_TRANS;       /* SDR */
    spimd_reg.adb    = SPIBSC_1BIT;
    spimd_reg.addr   = addr;

    ret = spibsc_transfer(&spimd_reg);
    if (ret == false) {
        return ret;
    }

    ret = _busy_wait();

    return ret;
}

bool FlashAccess::_PageProgram(uint32_t addr, uint8_t * buf, int32_t size) {
    bool ret;

    /* ---- Write enable   ---- */
    ret = _WriteEnable();      /* WREN Command */
    if (ret == false) {
        return ret;
    }

    /* ----------- 1. Command, Address ---------------*/
    /* ---- spimd_reg init ---- */
    clear_spimd_reg(&spimd_reg);

    /* ---- command ---- */
    spimd_reg.cde    = SPIBSC_OUTPUT_ENABLE;
    spimd_reg.cdb    = SPIBSC_1BIT;
    spimd_reg.cmd    = SFLASHCMD_PAGE_PROGRAM;

    /* ---- address ---- */
    spimd_reg.ade    = SPIBSC_OUTPUT_ADDR_24;
    spimd_reg.addre  = SPIBSC_SDR_TRANS;       /* SDR */
    spimd_reg.adb    = SPIBSC_1BIT;
    spimd_reg.addr   = addr;

    /* ---- Others ---- */
    spimd_reg.sslkp  = SPIBSC_SPISSL_KEEP;     /* SPBSSL level */

    ret = spibsc_transfer(&spimd_reg);         /* Command,Address */
    if (ret == false) {
        return ret;
    }

    /* ----------- 2. Data ---------------*/
    ret = data_send(SPIBSC_1BIT, SPIBSC_SPISSL_NEGATE, buf, size);
    if (ret == false) {
        return ret;
    }

    ret = _busy_wait();

    return ret;
}

bool FlashAccess::_Read(uint32_t addr, uint8_t * buf, int32_t size) {
    bool ret;

    /* ----------- 1. Command, Address ---------------*/
    /* ---- spimd_reg init ---- */
    clear_spimd_reg(&spimd_reg);

    /* ---- command ---- */
    spimd_reg.cde    = SPIBSC_OUTPUT_ENABLE;
    spimd_reg.cdb    = SPIBSC_1BIT;
    spimd_reg.cmd    = SFLASHCMD_READ;

    /* ---- address ---- */
    spimd_reg.ade    = SPIBSC_OUTPUT_ADDR_24;
    spimd_reg.addre  = SPIBSC_SDR_TRANS;       /* SDR */
    spimd_reg.adb    = SPIBSC_1BIT;
    spimd_reg.addr   = addr;

    /* ---- Others ---- */
    spimd_reg.sslkp  = SPIBSC_SPISSL_KEEP;     /* SPBSSL level */

    ret = spibsc_transfer(&spimd_reg);         /* Command,Address */
    if (ret == false) {
        return ret;
    }

    /* ----------- 2. Data ---------------*/
    ret = data_recv(SPIBSC_1BIT, SPIBSC_SPISSL_NEGATE, buf, size);

    return ret;
}

bool FlashAccess::_WriteEnable(void) {
    bool ret;

    /* ---- spimd_reg init ---- */
    clear_spimd_reg(&spimd_reg);

    /* ---- command ---- */
    spimd_reg.cde    = SPIBSC_OUTPUT_ENABLE;
    spimd_reg.cdb    = SPIBSC_1BIT;
    spimd_reg.cmd    = SFLASHCMD_WRITE_ENABLE;

    ret = spibsc_transfer(&spimd_reg);

    return ret;
}

bool FlashAccess::_busy_wait(void) {
    bool ret;
    uint8_t st_reg;

    while (1) {
        ret = _read_register(SFLASHCMD_READ_STATUS_REG, &st_reg);
        if (ret == false) {
            break;
        }
        if ((st_reg & STREG_BUSY_BIT) == 0) {
            break;
        }
    }

    return ret;
}

bool FlashAccess::_read_register(uint8_t cmd, uint8_t * status) {
    bool ret;

    /* ---- spimd_reg init ---- */
    clear_spimd_reg(&spimd_reg);

    /* ---- command ---- */
    spimd_reg.cde    = SPIBSC_OUTPUT_ENABLE;
    spimd_reg.cdb    = SPIBSC_1BIT;
    spimd_reg.cmd    = cmd;

    /* ---- Others ---- */
    spimd_reg.sslkp  = SPIBSC_SPISSL_NEGATE;   /* SPBSSL level */
    spimd_reg.spire  = SPIBSC_SPIDATA_ENABLE;  /* read enable/disable */
    spimd_reg.spiwe  = SPIBSC_SPIDATA_ENABLE;  /* write enable/disable */

    /* ---- data ---- */
    spimd_reg.spide  = SPIBSC_OUTPUT_SPID_8;   /* Enable(8bit) */
    spimd_reg.spidre = SPIBSC_SDR_TRANS;       /* SDR */
    spimd_reg.spidb  = SPIBSC_1BIT;
    spimd_reg.smwdr[0] = 0x00;                 /* Output 0 in read status */
    spimd_reg.smwdr[1] = 0x00;                 /* Output 0 in read status */

    ret = spibsc_transfer(&spimd_reg);
    if (ret != false) {
        *status = (uint8_t)(spimd_reg.smrdr[0]);   /* Data[7:0]  */
    }

    return ret;
}

bool FlashAccess::data_send(uint32_t bit_width, uint32_t spbssl_level, uint8_t * buf, int32_t size) {
    bool ret = true;
    int32_t unit;
    uint8_t  *buf_b;
    uint16_t *buf_s;
    uint32_t *buf_l;

    /* ---- spimd_reg init ---- */
    clear_spimd_reg(&spimd_reg);

    /* ---- Others ---- */
    spimd_reg.sslkp  = SPIBSC_SPISSL_KEEP;     /* SPBSSL level */
    spimd_reg.spiwe  = SPIBSC_SPIDATA_ENABLE;  /* write enable/disable */

    /* ---- data ---- */
    spimd_reg.spidb = bit_width;
    spimd_reg.spidre= SPIBSC_SDR_TRANS;        /* SDR */

    if (((uint32_t)size & 0x3)  == 0) {
        spimd_reg.spide = SPIBSC_OUTPUT_SPID_32;  /* Enable(32bit) */
        unit = 4;
    } else if (((uint32_t)size & 0x1) == 0) {
        spimd_reg.spide = SPIBSC_OUTPUT_SPID_16;  /* Enable(16bit) */
        unit = 2;
    } else {
        spimd_reg.spide = SPIBSC_OUTPUT_SPID_8;   /* Enable(8bit) */
        unit = 1;
    }

    while (size > 0) {
        if (unit == 1) {
            buf_b = (uint8_t *)buf;
            spimd_reg.smwdr[0] = (uint32_t)(((uint32_t)*buf_b) & 0x000000FF);
        } else if (unit == 2) {
            buf_s = (uint16_t *)buf;
            spimd_reg.smwdr[0] = (uint32_t)(((uint32_t)*buf_s) & 0x0000FFFF);
        } else if (unit == 4) {
            buf_l = (uint32_t *)buf;
            spimd_reg.smwdr[0] = (uint32_t)(((uint32_t)(*buf_l)) & 0xfffffffful);
        } else {
            /* Do Nothing */
        }

        buf  += unit;
        size -= unit;

        if (size <= 0) {
            spimd_reg.sslkp = spbssl_level;
        }

        ret = spibsc_transfer(&spimd_reg);    /* Data */
        if (ret == false) {
            return ret;
        }
    }

    return ret;
}

bool FlashAccess::data_recv(uint32_t bit_width, uint32_t spbssl_level, uint8_t * buf, int32_t size) {
    bool ret = true;
    int32_t unit;
    uint8_t  *buf_b;
    uint16_t *buf_s;
    uint32_t *buf_l;

    /* ---- spimd_reg init ---- */
    clear_spimd_reg(&spimd_reg);

    /* ---- Others ---- */
    spimd_reg.sslkp  = SPIBSC_SPISSL_KEEP;     /* SPBSSL level */
    spimd_reg.spire  = SPIBSC_SPIDATA_ENABLE;  /* read enable/disable */

    /* ---- data ---- */
    spimd_reg.spidb  = bit_width;
    spimd_reg.spidre = SPIBSC_SDR_TRANS;       /* SDR */

    if (((uint32_t)size & 0x3) == 0) {
        spimd_reg.spide = SPIBSC_OUTPUT_SPID_32;  /* Enable(32bit) */
        unit = 4;
    } else if (((uint32_t)size & 0x1) == 0) {
        spimd_reg.spide = SPIBSC_OUTPUT_SPID_16;  /* Enable(16bit) */
        unit = 2;
    } else {
        spimd_reg.spide = SPIBSC_OUTPUT_SPID_8;   /* Enable(8bit) */
        unit = 1;
    }

    while (size > 0) {
        if (unit >= size) {
            spimd_reg.sslkp = spbssl_level;
        }

        ret = spibsc_transfer(&spimd_reg);     /* Data */
        if (ret == false) {
            return ret;
        }

        if (unit == 1) {
            buf_b = (uint8_t *)buf;
            *buf_b = (uint8_t)((spimd_reg.smrdr[0]) & 0x000000fful);
        } else if (unit ==  2) {
            buf_s = (uint16_t *)buf;
            *buf_s = (uint16_t)((spimd_reg.smrdr[0]) & 0x0000fffful);
        } else if (unit == 4) {
            buf_l = (uint32_t *)buf;
            *buf_l = (uint32_t)((spimd_reg.smrdr[0]) & 0xfffffffful);
        } else {
            /* Do Nothing */
        }

        buf  += unit;
        size -= unit;
    }

    return ret;
}

void FlashAccess::spi_mode(void) {
    volatile uint32_t dummy_read_32;

    if (RegRead_32(&SPIBSC->CMNCR, SPIBSC_CMNCR_MD_SHIFT, SPIBSC_CMNCR_MD) != SPIBSC_CMNCR_MD_SPI) {
        /* ==== Change the MMU translation table SPI Multi-I/O bus space settings
                for use in SPI operating mode ==== */
        change_mmu_ttbl_spibsc(0);

        /* ==== Cleaning and invalidation of cache ==== */
        cache_control();

        /* ==== Switch to SPI operating mode ==== */
        spibsc_stop();

        dummy_read_32 = SPIBSC->CMNCR; /* dummy read */
        /* SPI Mode */
        RegWwrite_32(&SPIBSC->CMNCR, SPIBSC_CMNCR_MD_SPI, SPIBSC_CMNCR_MD_SHIFT, SPIBSC_CMNCR_MD);
        dummy_read_32 = SPIBSC->CMNCR; /* dummy read */

    }
    (void)dummy_read_32;
}

void FlashAccess::ex_mode(void) {
    volatile uint32_t dummy_read_32;

    if (RegRead_32(&SPIBSC->CMNCR, SPIBSC_CMNCR_MD_SHIFT, SPIBSC_CMNCR_MD) != SPIBSC_CMNCR_MD_EXTRD) {
        /* ==== Switch to external address space read mode and clear SPIBSC read cache ==== */
        spibsc_stop();

        /* Flush SPIBSC's read cache */
        RegWwrite_32(&SPIBSC->DRCR, SPIBSC_DRCR_RCF_EXE, SPIBSC_DRCR_RCF_SHIFT, SPIBSC_DRCR_RCF);
        dummy_read_32 = SPIBSC->DRCR;  /* dummy read */

        /* External address space read mode */
        RegWwrite_32(&SPIBSC->CMNCR, SPIBSC_CMNCR_MD_EXTRD, SPIBSC_CMNCR_MD_SHIFT, SPIBSC_CMNCR_MD);
        dummy_read_32 = SPIBSC->CMNCR; /* dummy read */

        /* ==== Change the MMU translation table SPI Multi-I/O bus space settings
                for use in external address space read mode ==== */
        change_mmu_ttbl_spibsc(1);

        /* ==== Cleaning and invalidation of cache ==== */
        cache_control();
    }
    (void)dummy_read_32;
}

void FlashAccess::clear_spimd_reg(st_spibsc_spimd_reg_t * regset) {
    /* ---- command ---- */
    regset->cde    = SPIBSC_OUTPUT_DISABLE;
    regset->cdb    = SPIBSC_1BIT;
    regset->cmd    = 0x00;

    /* ---- optional command ---- */
    regset->ocde   = SPIBSC_OUTPUT_DISABLE;
    regset->ocdb   = SPIBSC_1BIT;
    regset->ocmd   = 0x00;

    /* ---- address ---- */
    regset->ade    = SPIBSC_OUTPUT_DISABLE;
    regset->addre  = SPIBSC_SDR_TRANS;       /* SDR */
    regset->adb    = SPIBSC_1BIT;
    regset->addr   = 0x00000000;

    /* ---- option data ---- */
    regset->opde   = SPIBSC_OUTPUT_DISABLE;
    regset->opdre  = SPIBSC_SDR_TRANS;       /* SDR */
    regset->opdb   = SPIBSC_1BIT;
    regset->opd[0] = 0x00;    /* OPD3 */
    regset->opd[1] = 0x00;    /* OPD2 */
    regset->opd[2] = 0x00;    /* OPD1 */
    regset->opd[3] = 0x00;    /* OPD0 */

    /* ---- dummy cycle ---- */
    regset->dme    = SPIBSC_DUMMY_CYC_DISABLE;
    regset->dmdb   = SPIBSC_1BIT;
    regset->dmcyc  = SPIBSC_DUMMY_1CYC;

    /* ---- data ---- */
    regset->spide  = SPIBSC_OUTPUT_DISABLE;
    regset->spidre = SPIBSC_SDR_TRANS;       /* SDR */
    regset->spidb  = SPIBSC_1BIT;

    /* ---- Others ---- */
    regset->sslkp  = SPIBSC_SPISSL_NEGATE;   /* SPBSSL level */
    regset->spire  = SPIBSC_SPIDATA_DISABLE; /* read enable/disable */
    regset->spiwe  = SPIBSC_SPIDATA_DISABLE; /* write enable/disable */
}

bool FlashAccess::spibsc_transfer(st_spibsc_spimd_reg_t * regset) {
    if (RegRead_32(&SPIBSC->CMNCR, SPIBSC_CMNCR_MD_SHIFT, SPIBSC_CMNCR_MD) != SPIBSC_CMNCR_MD_SPI) {
        if (RegRead_32(&SPIBSC->CMNSR, SPIBSC_CMNSR_SSLF_SHIFT, SPIBSC_CMNSR_SSLF) != SPIBSC_SSL_NEGATE) {
            return false;
        }
        /* SPI Mode */
        RegWwrite_32(&SPIBSC->CMNCR, SPIBSC_CMNCR_MD_SPI, SPIBSC_CMNCR_MD_SHIFT, SPIBSC_CMNCR_MD);
    }

    if (RegRead_32(&SPIBSC->CMNSR, SPIBSC_CMNSR_TEND_SHIFT, SPIBSC_CMNSR_TEND) != SPIBSC_TRANS_END) {
        return false;
    }

    /* ---- Command ---- */
    /* Enable/Disable */
    RegWwrite_32(&SPIBSC->SMENR, regset->cde, SPIBSC_SMENR_CDE_SHIFT, SPIBSC_SMENR_CDE);
    if (regset->cde != SPIBSC_OUTPUT_DISABLE) {
        /* Command */
        RegWwrite_32(&SPIBSC->SMCMR, regset->cmd, SPIBSC_SMCMR_CMD_SHIFT, SPIBSC_SMCMR_CMD);
        /* Single/Dual/Quad */
        RegWwrite_32(&SPIBSC->SMENR, regset->cdb, SPIBSC_SMENR_CDB_SHIFT, SPIBSC_SMENR_CDB);
    }

    /* ---- Option Command ---- */
    /* Enable/Disable */
    RegWwrite_32(&SPIBSC->SMENR, regset->ocde, SPIBSC_SMENR_OCDE_SHIFT, SPIBSC_SMENR_OCDE);
    if (regset->ocde != SPIBSC_OUTPUT_DISABLE) {
        /* Option Command */
        RegWwrite_32(&SPIBSC->SMCMR, regset->ocmd, SPIBSC_SMCMR_OCMD_SHIFT, SPIBSC_SMCMR_OCMD);
        /* Single/Dual/Quad */
        RegWwrite_32(&SPIBSC->SMENR, regset->ocdb, SPIBSC_SMENR_OCDB_SHIFT, SPIBSC_SMENR_OCDB);
    }

    /* ---- Address ---- */
    /* Enable/Disable */
    RegWwrite_32(&SPIBSC->SMENR, regset->ade, SPIBSC_SMENR_ADE_SHIFT, SPIBSC_SMENR_ADE);
    if (regset->ade != SPIBSC_OUTPUT_DISABLE) {
        /* Address */
        RegWwrite_32(&SPIBSC->SMADR, regset->addr, SPIBSC_SMADR_ADR_SHIFT, SPIBSC_SMADR_ADR);
        /* Single/Dual/Quad */
        RegWwrite_32(&SPIBSC->SMENR, regset->adb, SPIBSC_SMENR_ADB_SHIFT, SPIBSC_SMENR_ADB);
    }

    /* ---- Option Data ---- */
    /* Enable/Disable */
    RegWwrite_32(&SPIBSC->SMENR, regset->opde, SPIBSC_SMENR_OPDE_SHIFT, SPIBSC_SMENR_OPDE);
    if (regset->opde != SPIBSC_OUTPUT_DISABLE) {
        /* Option Data */
        RegWwrite_32(&SPIBSC->SMOPR, regset->opd[0], SPIBSC_SMOPR_OPD3_SHIFT, SPIBSC_SMOPR_OPD3);
        RegWwrite_32(&SPIBSC->SMOPR, regset->opd[1], SPIBSC_SMOPR_OPD2_SHIFT, SPIBSC_SMOPR_OPD2);
        RegWwrite_32(&SPIBSC->SMOPR, regset->opd[2], SPIBSC_SMOPR_OPD1_SHIFT, SPIBSC_SMOPR_OPD1);
        RegWwrite_32(&SPIBSC->SMOPR, regset->opd[3], SPIBSC_SMOPR_OPD0_SHIFT, SPIBSC_SMOPR_OPD0);
        /* Single/Dual/Quad */
        RegWwrite_32(&SPIBSC->SMENR, regset->opdb, SPIBSC_SMENR_OPDB_SHIFT, SPIBSC_SMENR_OPDB);
    }

    /* ---- Dummy ---- */
     /* Enable/Disable */
     RegWwrite_32(&SPIBSC->SMENR, regset->dme, SPIBSC_SMENR_DME_SHIFT, SPIBSC_SMENR_DME);
     if (regset->dme != SPIBSC_DUMMY_CYC_DISABLE) {
         RegWwrite_32(&SPIBSC->SMDMCR, regset->dmdb, SPIBSC_SMDMCR_DMDB_SHIFT, SPIBSC_SMDMCR_DMDB);
         /* Dummy Cycle */
         RegWwrite_32(&SPIBSC->SMDMCR, regset->dmcyc, SPIBSC_SMDMCR_DMCYC_SHIFT, SPIBSC_SMDMCR_DMCYC);
     }

    /* ---- Data ---- */
    /* Enable/Disable */
    RegWwrite_32(&SPIBSC->SMENR, regset->spide, SPIBSC_SMENR_SPIDE_SHIFT, SPIBSC_SMENR_SPIDE);
    if (regset->spide != SPIBSC_OUTPUT_DISABLE) {
        if (SPIBSC_OUTPUT_SPID_8 == regset->spide) {
            if (RegRead_32(&SPIBSC0.CMNCR, SPIBSC_CMNCR_BSZ_SHIFT, SPIBSC_CMNCR_BSZ) == SPIBSC_CMNCR_BSZ_SINGLE) {
                SPIBSC->SMWDR0.UINT8[0] = (uint8_t)(regset->smwdr[0]);
            } else {
                SPIBSC->SMWDR0.UINT16[0] = (uint16_t)(regset->smwdr[0]);
            }
        } else if (regset->spide == SPIBSC_OUTPUT_SPID_16) {
            if (RegRead_32(&SPIBSC0.CMNCR, SPIBSC_CMNCR_BSZ_SHIFT, SPIBSC_CMNCR_BSZ) == SPIBSC_CMNCR_BSZ_SINGLE) {
                SPIBSC->SMWDR0.UINT16[0] = (uint16_t)(regset->smwdr[0]);
            } else {
                SPIBSC->SMWDR0.UINT32 = regset->smwdr[0];
            }
        } else if (regset->spide == SPIBSC_OUTPUT_SPID_32) {
            if (RegRead_32(&SPIBSC0.CMNCR, SPIBSC_CMNCR_BSZ_SHIFT, SPIBSC_CMNCR_BSZ) == SPIBSC_CMNCR_BSZ_SINGLE) {
                SPIBSC->SMWDR0.UINT32 = (uint32_t)(regset->smwdr[0]);
            } else {
                SPIBSC->SMWDR0.UINT32 = (uint32_t)(regset->smwdr[0]);
                SPIBSC->SMWDR1.UINT32 = (uint32_t)(regset->smwdr[1]);  /* valid in two serial-flash */
            }
        } else {
            /* none */
        }

        /* Single/Dual/Quad */
        RegWwrite_32(&SPIBSC->SMENR, regset->spidb, SPIBSC_SMENR_SPIDB_SHIFT, SPIBSC_SMENR_SPIDB);
    }

    RegWwrite_32(&SPIBSC->SMCR, regset->sslkp, SPIBSC_SMCR_SSLKP_SHIFT, SPIBSC_SMCR_SSLKP);

    if ((regset->spidb != SPIBSC_1BIT) && (regset->spide != SPIBSC_OUTPUT_DISABLE)) {
        if ((regset->spire == SPIBSC_SPIDATA_ENABLE) && (regset->spiwe == SPIBSC_SPIDATA_ENABLE)) {
            /* not set in same time */
            return false;
        }
    }

    RegWwrite_32(&SPIBSC->SMCR, regset->spire, SPIBSC_SMCR_SPIRE_SHIFT, SPIBSC_SMCR_SPIRE);
    RegWwrite_32(&SPIBSC->SMCR, regset->spiwe, SPIBSC_SMCR_SPIWE_SHIFT, SPIBSC_SMCR_SPIWE);

    /* SDR Transmission/DDR Transmission Setting */
    RegWwrite_32(&SPIBSC->SMDRENR, regset->addre, SPIBSC_SMDRENR_ADDRE_SHIFT, SPIBSC_SMDRENR_ADDRE);
    RegWwrite_32(&SPIBSC->SMDRENR, regset->opdre, SPIBSC_SMDRENR_OPDRE_SHIFT, SPIBSC_SMDRENR_OPDRE);
    RegWwrite_32(&SPIBSC->SMDRENR, regset->spidre, SPIBSC_SMDRENR_SPIDRE_SHIFT, SPIBSC_SMDRENR_SPIDRE);

    /* execute after setting SPNDL bit */
    RegWwrite_32(&SPIBSC->SMCR, SPIBSC_SPI_ENABLE, SPIBSC_SMCR_SPIE_SHIFT, SPIBSC_SMCR_SPIE);

    /* wait for transfer-start */
    while (RegRead_32(&SPIBSC->CMNSR, SPIBSC_CMNSR_TEND_SHIFT, SPIBSC_CMNSR_TEND) != SPIBSC_TRANS_END) {
        /* wait for transfer-end */
    }

    if (SPIBSC_OUTPUT_SPID_8 == regset->spide) {
        if (RegRead_32(&SPIBSC0.CMNCR, SPIBSC_CMNCR_BSZ_SHIFT, SPIBSC_CMNCR_BSZ) == SPIBSC_CMNCR_BSZ_SINGLE) {
            regset->smrdr[0] = SPIBSC->SMRDR0.UINT8[0];
        } else {
            regset->smrdr[0] = SPIBSC->SMRDR0.UINT16[0];        /* valid in two serial-flash  */
        }
    } else if (regset->spide == SPIBSC_OUTPUT_SPID_16) {
        if (RegRead_32(&SPIBSC0.CMNCR, SPIBSC_CMNCR_BSZ_SHIFT, SPIBSC_CMNCR_BSZ) == SPIBSC_CMNCR_BSZ_SINGLE) {
            regset->smrdr[0] = SPIBSC->SMRDR0.UINT16[0];
        } else {
            regset->smrdr[0] = SPIBSC->SMRDR0.UINT32;           /* valid in two serial-flash  */
        }
    } else if (regset->spide == SPIBSC_OUTPUT_SPID_32) {
        if (RegRead_32(&SPIBSC0.CMNCR, SPIBSC_CMNCR_BSZ_SHIFT, SPIBSC_CMNCR_BSZ) == SPIBSC_CMNCR_BSZ_SINGLE) {
            regset->smrdr[0] = SPIBSC->SMRDR0.UINT32;
        } else {
            regset->smrdr[0] = SPIBSC->SMRDR0.UINT32;           /* valid in two serial-flash  */
            regset->smrdr[1] = SPIBSC->SMRDR1.UINT32;
        }
    } else {
        /* none */
    }

    return true;
}

uint32_t FlashAccess::RegRead_32(volatile uint32_t * ioreg, uint32_t shift, uint32_t mask) {
    uint32_t reg_value;

    reg_value = *ioreg;                        /* Read from register            */
    reg_value = (reg_value & mask) >> shift;   /* Clear other bit and Bit shift */

    return reg_value;
}

void FlashAccess::RegWwrite_32(volatile uint32_t * ioreg, uint32_t write_value, uint32_t shift, uint32_t mask) {
    uint32_t reg_value;

    reg_value = *ioreg;                                         /* Read from register */
    reg_value = (reg_value & (~mask)) | (write_value << shift); /* Modify value       */
    *ioreg    = reg_value;                                      /* Write to register  */
}

/** private **/

void FlashAccess::change_mmu_ttbl_spibsc(uint32_t type) {
    uint32_t index;               /* Loop variable: table index */
    mmu_ttbl_desc_section_t desc; /* Loop variable: descriptor */
    mmu_ttbl_desc_section_t * table = (mmu_ttbl_desc_section_t *)TTB;

    /* ==== Modify SPI Multi-I/O bus space settings in the MMU translation table ==== */
    for (index = (SPIBSC_ADDR_START >> 20); index <= (SPIBSC_ADDR_END >> 20); index++) {
        /* Modify memory attribute descriptor */
        if (type == 0) {         /* Spi */
            desc = table[index];
            desc_tbl[index - (SPIBSC_ADDR_START >> 20)] = desc;
            desc.AP1_0 = 0x0u;   /* AP[2:0] = b'000 (No access) */
            desc.AP2   = 0x0u;
            desc.XN    = 0x1u;   /* XN = 1 (Execute never) */
        } else {                 /* Xip */
            desc = desc_tbl[index - (SPIBSC_ADDR_START >> 20)];
        }
        /* Write descriptor back to translation table */
        table[index] = desc;
    }
}

void FlashAccess::spibsc_stop(void) {
    if (((SPIBSC->DRCR & SPIBSC_DRCR_RBE)  != 0) &&
        ((SPIBSC->DRCR & SPIBSC_DRCR_SSLE) != 0)) {
        RegWwrite_32(&SPIBSC->DRCR, 1, SPIBSC_DRCR_SSLN_SHIFT, SPIBSC_DRCR_SSLN);
    }

    while (RegRead_32(&SPIBSC->CMNSR, SPIBSC_CMNSR_SSLF_SHIFT, SPIBSC_CMNSR_SSLF) != SPIBSC_SSL_NEGATE) {
        ;
    }

    while (RegRead_32(&SPIBSC->CMNSR, SPIBSC_CMNSR_TEND_SHIFT, SPIBSC_CMNSR_TEND) != SPIBSC_TRANS_END) {
        ;
    }
}

#ifndef __STATIC_FORCEINLINE
  #if   defined ( __CC_ARM )
    #define __STATIC_FORCEINLINE  static __forceinline
  #elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #define __STATIC_FORCEINLINE  __attribute__((always_inline)) static __inline
  #elif defined ( __GNUC__ )
    #define __STATIC_FORCEINLINE  __attribute__((always_inline)) static __inline
  #elif defined ( __ICCARM__ )
    #define __STATIC_FORCEINLINE  _Pragma("inline=forced") static inline
  #endif
#endif

#if   defined ( __CC_ARM )
__STATIC_FORCEINLINE __ASM void L1C_CleanInvalidateCache_sforce(uint32_t op) {
        ARM

        PUSH    {R4-R11}

        MRC     p15, 1, R6, c0, c0, 1      // Read CLIDR
        ANDS    R3, R6, #0x07000000        // Extract coherency level
        MOV     R3, R3, LSR #23            // Total cache levels << 1
        BEQ     Finished                   // If 0, no need to clean

        MOV     R10, #0                    // R10 holds current cache level << 1
Loop1   ADD     R2, R10, R10, LSR #1       // R2 holds cache "Set" position
        MOV     R1, R6, LSR R2             // Bottom 3 bits are the Cache-type for this level
        AND     R1, R1, #7                 // Isolate those lower 3 bits
        CMP     R1, #2
        BLT     Skip                       // No cache or only instruction cache at this level

        MCR     p15, 2, R10, c0, c0, 0     // Write the Cache Size selection register
        ISB                                // ISB to sync the change to the CacheSizeID reg
        MRC     p15, 1, R1, c0, c0, 0      // Reads current Cache Size ID register
        AND     R2, R1, #7                 // Extract the line length field
        ADD     R2, R2, #4                 // Add 4 for the line length offset (log2 16 bytes)
        LDR     R4, =0x3FF
        ANDS    R4, R4, R1, LSR #3         // R4 is the max number on the way size (right aligned)
        CLZ     R5, R4                     // R5 is the bit position of the way size increment
        LDR     R7, =0x7FFF
        ANDS    R7, R7, R1, LSR #13        // R7 is the max number of the index size (right aligned)

Loop2   MOV     R9, R4                     // R9 working copy of the max way size (right aligned)

Loop3   ORR     R11, R10, R9, LSL R5       // Factor in the Way number and cache number into R11
        ORR     R11, R11, R7, LSL R2       // Factor in the Set number
        CMP     R0, #0
        BNE     Dccsw
        MCR     p15, 0, R11, c7, c6, 2     // DCISW. Invalidate by Set/Way
        B       cont
Dccsw   CMP     R0, #1
        BNE     Dccisw
        MCR     p15, 0, R11, c7, c10, 2    // DCCSW. Clean by Set/Way
        B       cont
Dccisw  MCR     p15, 0, R11, c7, c14, 2    // DCCISW. Clean and Invalidate by Set/Way
cont    SUBS    R9, R9, #1                 // Decrement the Way number
        BGE     Loop3
        SUBS    R7, R7, #1                 // Decrement the Set number
        BGE     Loop2
Skip    ADD     R10, R10, #2               // Increment the cache number
        CMP     R3, R10
        BGT     Loop1

Finished
        DSB
        POP    {R4-R11}
        BX     lr
}

#elif (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) || defined ( __GNUC__ )
__STATIC_FORCEINLINE void L1C_CleanInvalidateCache_sforce(uint32_t op) {
  __ASM volatile(
    "        PUSH    {R4-R11}                   \n"

    "        MRC     p15, 1, R6, c0, c0, 1      \n" // Read CLIDR
    "        ANDS    R3, R6, #0x07000000        \n" // Extract coherency level
    "        MOV     R3, R3, LSR #23            \n" // Total cache levels << 1
    "        BEQ     Finished                   \n" // If 0, no need to clean

    "        MOV     R10, #0                    \n" // R10 holds current cache level << 1
    "Loop1:  ADD     R2, R10, R10, LSR #1       \n" // R2 holds cache "Set" position
    "        MOV     R1, R6, LSR R2             \n" // Bottom 3 bits are the Cache-type for this level
    "        AND     R1, R1, #7                 \n" // Isolate those lower 3 bits
    "        CMP     R1, #2                     \n"
    "        BLT     Skip                       \n" // No cache or only instruction cache at this level

    "        MCR     p15, 2, R10, c0, c0, 0     \n" // Write the Cache Size selection register
    "        ISB                                \n" // ISB to sync the change to the CacheSizeID reg
    "        MRC     p15, 1, R1, c0, c0, 0      \n" // Reads current Cache Size ID register
    "        AND     R2, R1, #7                 \n" // Extract the line length field
    "        ADD     R2, R2, #4                 \n" // Add 4 for the line length offset (log2 16 bytes)
    "        LDR     R4, =0x3FF                 \n"
    "        ANDS    R4, R4, R1, LSR #3         \n" // R4 is the max number on the way size (right aligned)
    "        CLZ     R5, R4                     \n" // R5 is the bit position of the way size increment
    "        LDR     R7, =0x7FFF                \n"
    "        ANDS    R7, R7, R1, LSR #13        \n" // R7 is the max number of the index size (right aligned)

    "Loop2:  MOV     R9, R4                     \n" // R9 working copy of the max way size (right aligned)

    "Loop3:  ORR     R11, R10, R9, LSL R5       \n" // Factor in the Way number and cache number into R11
    "        ORR     R11, R11, R7, LSL R2       \n" // Factor in the Set number
    "        CMP     R0, #0                     \n"
    "        BNE     Dccsw                      \n"
    "        MCR     p15, 0, R11, c7, c6, 2     \n" // DCISW. Invalidate by Set/Way
    "        B       cont                       \n"
    "Dccsw:  CMP     R0, #1                     \n"
    "        BNE     Dccisw                     \n"
    "        MCR     p15, 0, R11, c7, c10, 2    \n" // DCCSW. Clean by Set/Way
    "        B       cont                       \n"
    "Dccisw: MCR     p15, 0, R11, c7, c14, 2    \n" // DCCISW. Clean and Invalidate by Set/Way
    "cont:   SUBS    R9, R9, #1                 \n" // Decrement the Way number
    "        BGE     Loop3                      \n"
    "        SUBS    R7, R7, #1                 \n" // Decrement the Set number
    "        BGE     Loop2                      \n"
    "Skip:   ADD     R10, R10, #2               \n" // Increment the cache number
    "        CMP     R3, R10                    \n"
    "        BGT     Loop1                      \n"

    "Finished:                                  \n"
    "        DSB                                \n"
    "        POP    {R4-R11}                      "
  );
}

#else
__STATIC_FORCEINLINE void __L1C_MaintainDCacheSetWay_sforce(uint32_t level, uint32_t maint) {
    register volatile uint32_t Dummy;
    register volatile uint32_t ccsidr;
    uint32_t num_sets;
    uint32_t num_ways;
    uint32_t shift_way;
    uint32_t log2_linesize;
    uint32_t log2_num_ways;

    Dummy = level << 1;
    /* set csselr, select ccsidr register */
    __set_CCSIDR(Dummy);
    /* get current ccsidr register */
    ccsidr = __get_CCSIDR();
    num_sets = ((ccsidr & 0x0FFFE000) >> 13) + 1;
    num_ways = ((ccsidr & 0x00001FF8) >> 3) + 1;
    log2_linesize = (ccsidr & 0x00000007) + 2 + 2;
    log2_num_ways = log2_up(num_ways);
    shift_way = 32 - log2_num_ways;
    for (int way = num_ways-1; way >= 0; way--) {
        for (int set = num_sets-1; set >= 0; set--) {
            Dummy = (level << 1) | (set << log2_linesize) | (way << shift_way);
            switch (maint) {
                case 0:
                    // DCISW. Invalidate by Set/Way
                    __ASM volatile("MCR p15, 0, %0, c7, c6, 2" : : "r"(Dummy) : "memory");
                    break;
                case 1:
                    // DCCSW. Clean by Set/Way
                    __ASM volatile("MCR p15, 0, %0, c7, c10, 2" : : "r"(Dummy) : "memory");
                    break;
                default:
                    // DCCISW. Clean and Invalidate by Set/Way
                    __ASM volatile("MCR p15, 0, %0, c7, c14, 2" : : "r"(Dummy) : "memory");
                    break;
            }
        }
    }
    __DMB();
}

__STATIC_FORCEINLINE void L1C_CleanInvalidateCache_sforce(uint32_t op) {
    register volatile uint32_t clidr;
    uint32_t cache_type;
    clidr =  __get_CLIDR();
    for (uint32_t i = 0; i<7; i++) {
        cache_type = (clidr >> i*3) & 0x7UL;
        if ((cache_type >= 2) && (cache_type <= 4)) {
            __L1C_MaintainDCacheSetWay_sforce(i, op);
        }
    }
}
#endif

#ifdef MBED_VERSION

void FlashAccess::cache_control(void) {
    unsigned int assoc;

    /* ==== Cleaning and invalidation of the L1 data cache ==== */
    L1C_CleanInvalidateCache_sforce(2);
    __DSB();

    /* ==== Cleaning and invalidation of the L2 cache ==== */
    if (PL310->AUX_CNT & (1<<16)) {
        assoc = 16;
    } else {
        assoc =  8;
    }
    PL310->INV_WAY = (1 << assoc) - 1;
    while(PL310->INV_WAY & ((1 << assoc) - 1)); // poll invalidate
    PL310->CACHE_SYNC = 0x0;

    /* ==== Invalidate all TLB entries ==== */
    __ca9u_inv_tlb_all();

    /* ==== Invalidate the L1 instruction cache ==== */
    __v7_inv_icache_all();
    __DSB();
    __ISB();
}

#else  // mbed-os 5.6.4

void FlashAccess::cache_control(void) {
    unsigned int assoc;

    /* ==== Cleaning and invalidation of the L1 data cache ==== */
    L1C_CleanInvalidateCache_sforce(2);
    __DSB();

    /* ==== Cleaning and invalidation of the L2 cache ==== */
    if (L2C_310->AUX_CNT & (1U << 16U)) {
        assoc = 16U;
    } else {
        assoc =  8U;
    }
    L2C_310->CLEAN_INV_WAY = (1U << assoc) - 1U;
    while (L2C_310->CLEAN_INV_WAY & ((1U << assoc) - 1U)); // poll invalidate
    L2C_310->CACHE_SYNC = 0x0;

    /* ==== Invalidate all TLB entries ==== */
    __set_TLBIALL(0);
    __DSB();     // ensure completion of the invalidation
    __ISB();     // ensure instruction fetch path sees new state

    /* ==== Invalidate the L1 instruction cache ==== */
    __set_ICIALLU(0);
    __DSB();     // ensure completion of the invalidation
    __ISB();     // ensure instruction fetch path sees new I cache state
}
#endif
