/*
 * Copyright (c) 2017 Erik Alveflo
 * Copyright (c) 2014 Michele Balistreri
 * Copyright (c) 2011 Alex Hornung
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "MRF24J40.h"

#include <SPI.h>

#define spi_preamble() noInterrupts(); _set_cs_pin(0);
#define spi_postamble() _set_cs_pin(1); interrupts();

Mrf24j40::Mrf24j40(int cs_pin, int reset_pin, int wake_pin)
  : _cs_pin(cs_pin)
  , _reset_pin(reset_pin)
  , _wake_pin(wake_pin)
{
}

void Mrf24j40::_set_cs_pin(bool level) {
  digitalWrite(_cs_pin, level);
}

void Mrf24j40::_set_reset_pin(bool level) {
  if (_reset_pin >= 0) {
    digitalWrite(_reset_pin, level);
  }
}

void Mrf24j40::_set_wake_pin(bool level) {
  if (_wake_pin >= 0) {
    digitalWrite(_wake_pin, level);
  }
}

void Mrf24j40::_write_long_addr(uint16_t addr, uint8_t write) {
  spi_write(((addr >> 3) & 0x7F) | 0x80);
  spi_write(((addr << 5) & 0xE0) | (write << 4));
}

void Mrf24j40::_write_short_addr(uint8_t addr, uint8_t write) {
  spi_write(((addr << 1) & 0x7E) | write);
}

uint8_t Mrf24j40::read_long_ctrl_reg(uint16_t addr) {
  spi_preamble();
  _write_long_addr(addr, 0);
  uint8_t value = spi_read();
  spi_postamble();

  return value;
}

uint8_t Mrf24j40::read_short_ctrl_reg(uint8_t addr) {
  spi_preamble();
  _write_short_addr(addr, 0);
  uint8_t value = spi_read();
  spi_postamble();
  return value;
}

void Mrf24j40::write_long_ctrl_reg(uint16_t addr, uint8_t value) {
  spi_preamble();
  _write_long_addr(addr, 1);
  spi_write(value);
  spi_postamble();
}

void Mrf24j40::write_short_ctrl_reg(uint8_t addr, uint8_t value) {
  spi_preamble();
  _write_short_addr(addr, 1);
  spi_write(value);
  spi_postamble();
}

void Mrf24j40::ie(void) {
  write_short_ctrl_reg(MRF24J40_INTCON, ~(TXNIE | RXIE | SECIE));
}

void Mrf24j40::pwr_reset(void) {
  write_short_ctrl_reg(SOFTRST, RSTPWR);
}

void Mrf24j40::bb_reset(void) {
  write_short_ctrl_reg(SOFTRST, RSTBB);
}

void Mrf24j40::mac_reset(void) {
  write_short_ctrl_reg(SOFTRST, RSTMAC);
}

void Mrf24j40::rf_reset(void) {
  uint8_t old = read_short_ctrl_reg(RFCTL);
  write_short_ctrl_reg(RFCTL, old | RFRST);
  write_short_ctrl_reg(RFCTL, old & ~RFRST);
  delay(2);
}

uint8_t Mrf24j40::get_pending_frame(void) {
  return (read_short_ctrl_reg(TXNCON) >> 4) & 0x01;
}

void Mrf24j40::rxfifo_flush(void) {
  write_short_ctrl_reg(RXFLUSH, (read_short_ctrl_reg(RXFLUSH) | _RXFLUSH));
}

void Mrf24j40::set_channel(int16_t ch) {
  write_long_ctrl_reg(RFCON0, CHANNEL(ch) | RFOPT(0x03));
  rf_reset();
}

void Mrf24j40::set_promiscuous(bool crc_check) {
  uint8_t w = NOACKRSP;
  if (!crc_check) {
    w |= ERRPKT;
  } else {
    w |= PROMI;
  }
  write_short_ctrl_reg(RXMCR, w);
}

void Mrf24j40::set_coordinator(void) {
  write_short_ctrl_reg(RXMCR, read_short_ctrl_reg(RXMCR) | PANCOORD);
}

void Mrf24j40::clear_coordinator(void) {
  write_short_ctrl_reg(RXMCR, read_short_ctrl_reg(RXMCR) & ~PANCOORD);
}

void Mrf24j40::set_pan(uint8_t *pan) {
  write_short_ctrl_reg(PANIDL, pan[0]);
  write_short_ctrl_reg(PANIDH, pan[1]);
}

void Mrf24j40::set_short_addr(uint8_t *addr) {
  write_short_ctrl_reg(SADRL, addr[0]);
  write_short_ctrl_reg(SADRH, addr[1]);
}

void Mrf24j40::set_eui(uint8_t *eui) {
  write_short_ctrl_reg(EADR0, eui[0]);
  write_short_ctrl_reg(EADR1, eui[1]);
  write_short_ctrl_reg(EADR2, eui[2]);
  write_short_ctrl_reg(EADR3, eui[3]);
  write_short_ctrl_reg(EADR4, eui[4]);
  write_short_ctrl_reg(EADR5, eui[5]);
  write_short_ctrl_reg(EADR6, eui[6]);
  write_short_ctrl_reg(EADR7, eui[7]);
}

void Mrf24j40::set_coordinator_short_addr(uint8_t *addr) {
  write_short_ctrl_reg(ASSOSADR0, addr[0]);
  write_short_ctrl_reg(ASSOSADR1, addr[1]);
}

void Mrf24j40::set_coordinator_eui(uint8_t *eui) {
  write_short_ctrl_reg(ASSOEADR0, eui[0]);
  write_short_ctrl_reg(ASSOEADR1, eui[1]);
  write_short_ctrl_reg(ASSOEADR2, eui[2]);
  write_short_ctrl_reg(ASSOEADR3, eui[3]);
  write_short_ctrl_reg(ASSOEADR4, eui[4]);
  write_short_ctrl_reg(ASSOEADR5, eui[5]);
  write_short_ctrl_reg(ASSOEADR6, eui[6]);
  write_short_ctrl_reg(ASSOEADR7, eui[7]);
}

void Mrf24j40::set_key(uint16_t address, uint8_t *key) {
  spi_preamble();
  _write_long_addr(address, 1);

  for (int16_t i = 0; i < 16; i++) {
    spi_write(key[i]);
  }

  spi_postamble();
}

void Mrf24j40::hard_reset(void) {
  _set_reset_pin(0);
  delayMicroseconds(192);
  _set_reset_pin(1);
  delayMicroseconds(192);
}

void Mrf24j40::soft_reset(void) {
  write_short_ctrl_reg(SOFTRST, (RSTPWR | RSTBB | RSTMAC));
  delayMicroseconds(192);
}

void Mrf24j40::_configure_pins(void) {
  pinMode(_cs_pin, OUTPUT);
  if (_reset_pin >= 0) {
    pinMode(_reset_pin, OUTPUT);
  }
  if (_wake_pin >= 0) {
    pinMode(_wake_pin, OUTPUT);
  }

  _set_cs_pin(1);
  _set_reset_pin(1);
  _set_wake_pin(1);
}

void Mrf24j40::initialize(void) {
  _configure_pins();

  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.begin();

  hard_reset();
  delay(2);

  soft_reset();

  write_short_ctrl_reg(PACON2, FIFOEN | TXONTS(0x18));
  write_short_ctrl_reg(TXSTBL, RFSTBL(9) | MSIFS(5));
  write_long_ctrl_reg(RFCON1, VCOOPT(0x01));
  write_long_ctrl_reg(RFCON2, PLLEN);
  write_long_ctrl_reg(RFCON6, _20MRECVR);
  write_long_ctrl_reg(RFCON7, SLPCLKSEL(0x02));
  write_long_ctrl_reg(RFCON8, RFVCO);
  write_long_ctrl_reg(SLPCON1, SLPCLKDIV(1) | CLKOUTDIS);
  write_short_ctrl_reg(RXFLUSH, (WAKEPAD | WAKEPOL));

  write_short_ctrl_reg(BBREG2, CCAMODE(0x02) | CCASTH(0x00));
  write_short_ctrl_reg(CCAEDTH, 0x60);

  write_short_ctrl_reg(BBREG6, RSSIMODE2);

  rxfifo_flush();

  ie();
}

void Mrf24j40::sleep(void) {
  write_short_ctrl_reg(WAKECON, IMMWAKE);

  uint8_t r = read_short_ctrl_reg(SLPACK);
  _set_wake_pin(0);

  pwr_reset();
  write_short_ctrl_reg(SLPACK, r | _SLPACK);
}

void Mrf24j40::wakeup(void) {
  _set_wake_pin(1);
  rf_reset();
}

void Mrf24j40::txpkt(uint8_t *frame, int16_t hdr_len, int16_t sec_hdr_len, int16_t payload_len) {
  int16_t frame_len = hdr_len + sec_hdr_len + payload_len;

  uint8_t w = read_short_ctrl_reg(TXNCON);
  w &= ~(TXNSECEN);

  if (IEEE_802_15_4_HAS_SEC(frame[0])) {
    w |= TXNSECEN;
  }

  if (IEEE_802_15_4_WANTS_ACK(frame[0])) {
    w |= TXNACKREQ;
  }

  spi_preamble();
  _write_long_addr(TXNFIFO, 1);
  spi_write(hdr_len);
  spi_write(frame_len);

  while (frame_len-- > 0) {
    spi_write(*frame++);
  }

  spi_postamble();

  write_short_ctrl_reg(TXNCON, w | TXNTRIG);
}

void Mrf24j40::set_cipher(uint8_t rxcipher, uint8_t txcipher) {
  write_short_ctrl_reg(SECCON0, RXCIPHER(rxcipher) | TXNCIPHER(txcipher));
}

bool Mrf24j40::rx_sec_fail(void) {
  bool rx_sec_fail = (read_short_ctrl_reg(RXSR) >> 2) & 0x01;
  write_short_ctrl_reg(RXSR, 0x00);
  return rx_sec_fail;
}

void Mrf24j40::sec_intcb(bool accept) {
  uint8_t w = read_short_ctrl_reg(SECCON0);

  w |= accept ? SECSTART : SECIGNORE;
  write_short_ctrl_reg(SECCON0, w);
}

int16_t Mrf24j40::txpkt_intcb(void) {
  uint8_t stat = read_short_ctrl_reg(TXSTAT);
  if (stat & TXNSTAT) {
    if (stat & CCAFAIL) {
      return EBUSY;
    } else {
      return EIO;
    }
  } else {
    return 0;
  }
}

int16_t Mrf24j40::rxpkt_intcb(uint8_t *buf, uint8_t *plqi, uint8_t *prssi) {
  write_short_ctrl_reg(BBREG1, read_short_ctrl_reg(BBREG1) | RXDECINV);

  spi_preamble();
  _write_long_addr(RXFIFO, 0);

  uint16_t flen = spi_read();

  for (uint16_t i = 0; i < flen; i++) {
    *buf++ = spi_read();
  }

  uint8_t lqi = spi_read();
  uint8_t rssi = spi_read();

  if (plqi != NULL) {
    *plqi = lqi;
  }

  if (prssi != NULL) {
    *prssi = rssi;
  }

  spi_postamble();

  rxfifo_flush();
  write_short_ctrl_reg(BBREG1, read_short_ctrl_reg(BBREG1) & ~RXDECINV);

  return flen;
}

int16_t Mrf24j40::int_tasks(void) {
  int16_t ret = 0;

  uint8_t stat = read_short_ctrl_reg(INTSTAT);

  if (stat & RXIF) {
    ret |= MRF24J40_INT_RX;
  }

  if (stat & TXNIF) {
    ret |= MRF24J40_INT_TX;
  }

  if (stat & SECIF) {
    ret |= MRF24J40_INT_SEC;
  }

  return ret;
}

void Mrf24j40::spi_write(uint8_t byte) {
  SPI.transfer(byte);
}

uint8_t Mrf24j40::spi_read(void) {
  return SPI.transfer(0xFF);
}