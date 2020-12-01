// +build nrf52 nrf52840

package machine

import (
	"device/nrf"
	"unsafe"
)

// SPI on the NRF.
type SPI struct {
	Bus *nrf.SPIM_Type
}

// There are 3 SPI interfaces on the NRF528xx.
var (
	SPI0 = SPI{Bus: nrf.SPIM0}
	SPI1 = SPI{Bus: nrf.SPIM1}
	SPI2 = SPI{Bus: nrf.SPIM2}
)

// SPIConfig is used to store config info for SPI.
type SPIConfig struct {
	Frequency uint32
	SCK       Pin
	SDO       Pin
	SDI       Pin
	LSBFirst  bool
	Mode      uint8
}

// Configure is intended to setup the SPI interface.
func (spi SPI) Configure(config SPIConfig) {
	// Disable bus to configure it
	spi.Bus.ENABLE.Set(nrf.SPIM_ENABLE_ENABLE_Disabled)

	// Pick a default frequency.
	if config.Frequency == 0 {
		config.Frequency = 4000000 // 4MHz
	}

	// set frequency
	var freq uint32
	switch {
	case config.Frequency >= 8000000:
		freq = nrf.SPIM_FREQUENCY_FREQUENCY_M8
	case config.Frequency >= 4000000:
		freq = nrf.SPIM_FREQUENCY_FREQUENCY_M4
	case config.Frequency >= 2000000:
		freq = nrf.SPIM_FREQUENCY_FREQUENCY_M2
	case config.Frequency >= 1000000:
		freq = nrf.SPIM_FREQUENCY_FREQUENCY_M1
	case config.Frequency >= 500000:
		freq = nrf.SPIM_FREQUENCY_FREQUENCY_K500
	case config.Frequency >= 250000:
		freq = nrf.SPIM_FREQUENCY_FREQUENCY_K250
	default: // below 250kHz, default to the lowest speed available
		freq = nrf.SPIM_FREQUENCY_FREQUENCY_K125
	}
	spi.Bus.FREQUENCY.Set(freq)

	var conf uint32

	// set bit transfer order
	if config.LSBFirst {
		conf = (nrf.SPIM_CONFIG_ORDER_LsbFirst << nrf.SPIM_CONFIG_ORDER_Pos)
	}

	// set mode
	switch config.Mode {
	case 0:
		conf &^= (nrf.SPIM_CONFIG_CPOL_ActiveHigh << nrf.SPIM_CONFIG_CPOL_Pos)
		conf &^= (nrf.SPIM_CONFIG_CPHA_Leading << nrf.SPIM_CONFIG_CPHA_Pos)
	case 1:
		conf &^= (nrf.SPIM_CONFIG_CPOL_ActiveHigh << nrf.SPIM_CONFIG_CPOL_Pos)
		conf |= (nrf.SPIM_CONFIG_CPHA_Trailing << nrf.SPIM_CONFIG_CPHA_Pos)
	case 2:
		conf |= (nrf.SPIM_CONFIG_CPOL_ActiveLow << nrf.SPIM_CONFIG_CPOL_Pos)
		conf &^= (nrf.SPIM_CONFIG_CPHA_Leading << nrf.SPIM_CONFIG_CPHA_Pos)
	case 3:
		conf |= (nrf.SPIM_CONFIG_CPOL_ActiveLow << nrf.SPIM_CONFIG_CPOL_Pos)
		conf |= (nrf.SPIM_CONFIG_CPHA_Trailing << nrf.SPIM_CONFIG_CPHA_Pos)
	default: // to mode
		conf &^= (nrf.SPIM_CONFIG_CPOL_ActiveHigh << nrf.SPIM_CONFIG_CPOL_Pos)
		conf &^= (nrf.SPIM_CONFIG_CPHA_Leading << nrf.SPIM_CONFIG_CPHA_Pos)
	}
	spi.Bus.CONFIG.Set(conf)

	// set pins
	if config.SCK == 0 && config.SDO == 0 && config.SDI == 0 {
		config.SCK = SPI0_SCK_PIN
		config.SDO = SPI0_SDO_PIN
		config.SDI = SPI0_SDI_PIN
	}
	spi.Bus.PSEL.SCK.Set(uint32(config.SCK))
	spi.Bus.PSEL.MOSI.Set(uint32(config.SDO))
	spi.Bus.PSEL.MISO.Set(uint32(config.SDI))

	// Re-enable bus now that it is configured.
	spi.Bus.ENABLE.Set(nrf.SPIM_ENABLE_ENABLE_Enabled)
}

// Transfer writes/reads a single byte using the SPI interface.
func (spi SPI) Transfer(w byte) (byte, error) {
	var wbuf, rbuf [1]byte
	wbuf[0] = w
	err := spi.Tx(wbuf[:], rbuf[:])
	return rbuf[0], err
}

// Tx handles read/write operation for SPI interface. Since SPI is a syncronous
// write/read interface, there must always be the same number of bytes written
// as bytes read. Therefore, if the number of bytes don't match it will be
// padded until they fit: if len(w) > len(r) the extra bytes received will be
// dropped and if len(w) < len(r) extra 0 bytes will be sent.
func (spi SPI) Tx(w, r []byte) error {
	// Unfortunately the hardware (on the nrf52832) only supports up to 255
	// bytes in the buffers, so if either w or r is longer than that the
	// transfer needs to be broken up in pieces.
	// The nrf52840 supports far larger buffers however, which isn't yet
	// supported.
	for len(r) != 0 || len(w) != 0 {
		// Prepare the SPI transfer: set the DMA pointers and lengths.
		if len(r) != 0 {
			spi.Bus.RXD.PTR.Set(uint32(uintptr(unsafe.Pointer(&r[0]))))
			n := uint32(len(r))
			if n > 255 {
				n = 255
			}
			spi.Bus.RXD.MAXCNT.Set(n)
			r = r[n:]
		}
		if len(w) != 0 {
			spi.Bus.TXD.PTR.Set(uint32(uintptr(unsafe.Pointer(&w[0]))))
			n := uint32(len(w))
			if n > 255 {
				n = 255
			}
			spi.Bus.TXD.MAXCNT.Set(n)
			w = w[n:]
		}

		// Do the transfer.
		// Note: this can be improved by not waiting until the transfer is
		// finished if the transfer is send-only (a common case).
		spi.Bus.TASKS_START.Set(1)
		for spi.Bus.EVENTS_END.Get() == 0 {
		}
		spi.Bus.EVENTS_END.Set(0)
	}

	return nil
}
