from argparse import ArgumentParser as parser

parse = parser(
	prog="pin to hex",
	description="convert an STM32 pin for a peripheral to hex"
)
flag =			{"action": "store_true"}
input_default =	{"action": "store", "type": int, "default": None}


if __name__ == "__main__":
	parse.add_argument('-af', **input_default)
	parse.add_argument('-pnum', **input_default)
	
	parse.add_argument('-tim', **flag)
	parse.add_argument('-uart', **flag)
	parse.add_argument('-i2c', **flag)
	parse.add_argument('-usb', **flag)
	parse.add_argument('-spi', **flag)
	arg = parse.parse_args()
	
	clocks = {
		"AHB1": 0,
		"AHB2": 1,
		"AHB3": 2,
		"APB1": 4,
		"APB2": 5
	}

	ports = {
		"A": 0, "B": 1, "C": 2, "D": 3,
		"E": 4, "F": 5, "G": 6, "H": 7,
		"I": 8, "J": 9, "K": 10
	}
	
	tims = {
		"TIM1": ("APB2", 0),     "TIM2": ("APB1", 0)
	}
	uarts = {
		"UART1": ("APB2", 4),    "UART2": ("APB1", 17),
		"UART3": ("APB1", 18),   "UART6": ("APB2", 5)
	}
	i2cs = {
		"I2C1": ("APB1", 21),    "I2C2": ("APB1", 22)
	}
	usbs = {
		"USB1": ("AHB1", 25)
	}
	spis = {
		"SPI1": ("APB2", 12),	"SPI2": ("APB1", 14)
	}
	
	while True:
		try:
			sub = 0
			if arg.tim:
				tim = input("tim: ") if not arg.pnum else arg.pnum
				try:    tim = tims[f"TIM{int(tim)}"]
				except: tim = tims[tim.upper()]
				clk, dev = tim
				clk = clocks[clk]
				channel = max((int(input("channel: ")) - 1), 0)
				sub |= channel & 0x7
			elif arg.uart:
				uart = input("uart: ") if not arg.pnum else arg.pnum
				try:    uart = uarts[f"UART{int(uart)}"]
				except: uart = uarts[uart.upper()]
				clk, dev = uart
				clk = clocks[clk]
			elif arg.i2c:
				i2c = input("i2c: ") if not arg.pnum else arg.pnum
				try:    i2c = i2cs[f"I2C{int(i2c)}"]
				except: i2c = i2cs[i2c.upper()]
				clk, dev = i2c
				clk = clocks[clk]
			elif arg.usb:
				usb = input("usb: ") if not arg.pnum else arg.pnum
				ulpi = input("ulpi?") != ""
				print("ulpi " + ("ON" if ulpi else "OFF"))
				try:
					clk, dev = usbs[f"USB{int(usb)}"]
					if ulpi: sub = (0x1 << 5) | ((dev + 1) & 0x1f)  # clock is always AHB1
				except:
					clk, dev = usbs[usb.upper()]
					if ulpi: sub = (0x1 << 5) | ((dev + 1) & 0x1f)  # clock is always AHB1
				clk = clocks[clk]
			elif arg.spi:
				spi = input("spi: ") if not arg.pnum else arg.pnum
				try:    spi = spis[f"SPI{int(spi)}"]
				except: spi = spis[spi.upper()]
				clk, dev = spi
				clk = clocks[clk]
			else:
				clk = input("clk: ")
				try:    clk = int(clk)
				except: clk = clocks[clk.upper()]
				dev = int(input("offset: "), base=16) >> 10
			alt = int(input("alt: ")) if not arg.af else arg.af
			pin = input("pin: ")
			port = int(ports[pin[0].upper()])
			pin = int(pin[1:])
			res = (
					((alt & 0xf) << 28)     |
					((pin & 0xf) << 24)     |
					((port & 0xf) << 20)    |
					((sub & 0xff) << 12)    |
					((dev & 0xff) << 4)		|
					((clk & 0xf) << 0)
			)
			#print(hex(sub))
			print(f"{res:#0{10}x}".upper().replace("X", "x"), end="\n\n")
		except KeyboardInterrupt:   exit(0)
		except Exception as e:      print(e); pass
