/*
 *
 * This file is part of the libopencm3 project.
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/quadspi.h>

/*  set clock prescaler
    note that the AHB clock is divided by prescaler + 1
    to obtain the quadspi clk frequency
*/
void quadspi_set_prescaler(uint8_t prescaler)
{
    uint32_t reg = QUADSPI_CR;
    reg &= ~(QUADSPI_CR_PRESCALE_MASK << QUADSPI_CR_PRESCALE_SHIFT);
    reg |= prescaler << QUADSPI_CR_PRESCALE_SHIFT;
    QUADSPI_CR = reg;
}

void quadspi_disable(void)
{
    QUADSPI_CR &= ~(QUADSPI_CR_EN);
}

void quadspi_enable(void)
{
    QUADSPI_CR |= QUADSPI_CR_EN;
}

void quadspi_enable_dma(void)
{
    QUADSPI_CR |= QUADSPI_CR_DMAEN;
}

void quadspi_disable_dma(void)
{
    QUADSPI_CR &= ~(QUADSPI_CR_DMAEN);
}

/*  set the number of bytes contained in the flash memory
    number of bytes = 2^(flash_size + 1)
    note that memory mapped mode can only address 256MB worth
    if using dual-flash mode, this should be the total of the two
*/
void quadspi_set_flash_size(uint8_t flash_size)
{
    uint32_t reg = QUADSPI_DCR;
    reg &= ~(QUADSPI_DCR_FSIZE_MASK << QUADSPI_DCR_FSIZE_SHIFT);
    reg |= flash_size << QUADSPI_DCR_FSIZE_SHIFT;
    QUADSPI_DCR = reg;
}

/*  set the amount of time nCS must remain high between commands
    note: actual delay will be high_cycles + 1
*/
void quadspi_set_cs_high_cyc(uint8_t high_cycles)
{
    uint32_t reg = QUADSPI_DCR;
    reg &= ~(QUADSPI_DCR_CSHT_MASK << QUADSPI_DCR_CSHT_SHIFT);
    reg |= high_cycles << QUADSPI_DCR_CSHT_SHIFT;
    QUADSPI_DCR = reg;
}

void quadspi_clk_idle_high(void)
{
    //Similar to spi mode 3
    QUADSPI_DCR |= QUADSPI_DCR_CKMODE;
}

void quadspi_clk_idle_low(void)
{
    //Similar to spi mode 0
    QUADSPI_DCR &= ~(QUADSPI_DCR_CKMODE);
}

void quadspi_set_instruction(uint8_t instr)
{
    uint32_t reg = QUADSPI_CCR;
    reg &= ~(QUADSPI_CCR_INST_MASK << QUADSPI_CCR_INST_SHIFT);
    reg |= instr << QUADSPI_CCR_INST_SHIFT;
    QUADSPI_CCR = reg;
}

/* --Set whether a mode is disabled (0), or occurs on one, two or four lines-- */
void quadspi_set_inst_mode(uint8_t imode)
{
    uint32_t reg = QUADSPI_CCR;
    reg &= ~(QUADSPI_CCR_IMODE_MASK << QUADSPI_CCR_IMODE_SHIFT);
    reg |= imode << QUADSPI_CCR_IMODE_SHIFT;
    QUADSPI_CCR = reg;
}

void quadspi_set_addr_mode(uint8_t amode)
{
    uint32_t reg = QUADSPI_CCR;
    reg &= ~(QUADSPI_CCR_ADMODE_MASK << QUADSPI_CCR_ADMODE_SHIFT);
    reg |= amode << QUADSPI_CCR_ADMODE_SHIFT;
    QUADSPI_CCR = reg;
}
void quadspi_set_alt_mode(uint8_t abmode)
{
    uint32_t reg = QUADSPI_CCR;
    reg &= ~(QUADSPI_CCR_ABMODE_MASK << QUADSPI_CCR_ABMODE_SHIFT);
    reg |= abmode << QUADSPI_CCR_ABMODE_SHIFT;
    QUADSPI_CCR = reg;
}

void quadspi_set_data_mode(uint8_t dmode)
{
    uint32_t reg = QUADSPI_CCR;
    reg &= ~(QUADSPI_CCR_DMODE_MASK << QUADSPI_CCR_DMODE_SHIFT);
    reg |= dmode << QUADSPI_CCR_DMODE_SHIFT;
    QUADSPI_CCR = reg;
}

/*  disable/enable sending instruction field once
*/
void quadspi_enable_instr_once(void)
{
    QUADSPI_CCR |= QUADSPI_CCR_SIOO;
}

void quadspi_disable_instr_once(void)
{
    QUADSPI_CCR &= ~(QUADSPI_CCR_SIOO);
}

/*  number of data bytes to be retrieved in polling and indirect modes
    actual number of bytes retrieved is data_len + 1
*/
void quadspi_set_data_len(uint32_t data_len)
{
    QUADSPI_DLR = data_len;
}

void quadspi_send_word(uint32_t data)
{
    QUADSPI_DR = data;
}

uint32_t quadspi_get_word(void)
{
    return QUADSPI_DR;
}

void quadspi_send_byte(uint8_t data)
{
    QUADSPI_BYTE_DR = data;
}

uint8_t quadspi_get_byte(void)
{
    return QUADSPI_BYTE_DR;
}

/*  set size of alt bytes to send
    00: 8-bit
    01: 16-bit
    10: 24-bit
    11: 32 bit
*/
void quadspi_set_alt_bytes_size(uint8_t absize)
{
    uint32_t reg = QUADSPI_CCR;
    reg &= ~(QUADSPI_CCR_ABSIZE_MASK << QUADSPI_CCR_ABSIZE_SHIFT);
    reg |= absize << QUADSPI_CCR_ABSIZE_SHIFT;
    QUADSPI_CCR = reg;
}

/*  configure the size of an address in QUADSPI_AR
    00: 8-bit
    01: 16-bit
    10: 24-bit
    11: 32 bit
*/
void quadspi_set_addr_size(uint8_t adsize)
{
    uint32_t reg = QUADSPI_CCR;
    reg &= ~(QUADSPI_CCR_ADSIZE_MASK << QUADSPI_CCR_ADSIZE_SHIFT);
    reg |= adsize << QUADSPI_CCR_ADSIZE_SHIFT;
    QUADSPI_CCR = reg;
}

/* set quadspi mode
00 = indirect write
01 = indirect read
10 = auto polling
11 = memory-mapped
NOTE: disable DMA before changing FMODE
BUSY must be 0 when changing mode
*/
void quadspi_set_func_mode(uint8_t fmode)
{
    uint32_t reg = QUADSPI_CCR;
    reg &= ~(QUADSPI_CCR_FMODE_MASK << QUADSPI_CCR_FMODE_SHIFT);
    reg |= fmode << QUADSPI_CCR_FMODE_SHIFT;
    QUADSPI_CCR = reg;
}

uint8_t quadspi_get_func_mode(void)
{
    uint32_t reg = QUADSPI_CCR >> QUADSPI_CCR_FMODE_SHIFT;
    reg &= QUADSPI_CCR_FMODE_MASK;
    return (uint8_t) reg;
}

/* From the manual: p 325
In order to assure enough “turn-around” time for changing the data signals
from output mode to input mode, there must be at least one dummy cycle when
using dual or quad mode to receive data from the Flash memory.
    Set number of dummy cycles needed for quadspi device
    0 indicates the phase is skipped
    a value of 1-31 indicates that many CLK cycles will be sent
    NOTE: dual or quad mode require at least one cycle
*/
void quadspi_set_dummy_cycles(uint8_t cycles)
{
    uint32_t reg = QUADSPI_CCR;
    reg &= ~(QUADSPI_CCR_DCYC_MASK << QUADSPI_CCR_DCYC_SHIFT);
    reg |= (cycles & QUADSPI_CCR_DCYC_MASK) << QUADSPI_CCR_DCYC_SHIFT;
    QUADSPI_CCR = reg;
}

void quadspi_set_fifo_thresh(uint8_t thresh)
{
    uint32_t reg = QUADSPI_CR;
    reg &= ~(QUADSPI_CR_FTHRES_MASK << QUADSPI_CR_FTHRES_SHIFT);
    reg |= (thresh & QUADSPI_CR_FTHRES_MASK) << QUADSPI_CR_FTHRES_SHIFT;
    QUADSPI_CR = reg;
}

bool quadspi_is_busy(void)
{
    return  (QUADSPI_CCR & QUADSPI_SR_BUSY) ? 1 : 0;
}

uint8_t quadspi_get_dummy_cycles(void)
{
    return (uint8_t) (QUADSPI_CCR >> QUADSPI_CCR_DCYC_SHIFT) & QUADSPI_CCR_DCYC_MASK;
}

void quadspi_set_addr(uint32_t addr)
{
    QUADSPI_AR = addr;
}
